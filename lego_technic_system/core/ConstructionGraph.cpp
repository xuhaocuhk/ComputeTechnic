//
// Created by 徐豪 on 2018/5/19.
//

#include <fstream>
#include <util/Timer.h>
#include "ConstructionGraph.h"
#include "util/util.h"
#include "configure/global_variables.h"
#include "IO/DataWriter.h"
#include "connector/BeamLayoutConnectorGenerator.h"

using std::cout;

ConstructionGraph::ConstructionGraph(vector<LayerGraph*>& layers){
    /* build layer nodes from layer graph */
    for(int i = 0; i < layers.size(); i++){
        Component* layerNode = new Component;
        layerNode->_id = i;
        layerNode->layerGraph = layers.at(i);
        this->components.push_back(layerNode);
    }

    /* build adjacency relationship */
    for(auto& comp_node : components){
        for(auto& adj_node : components){
            if(comp_node == adj_node)
                continue;
            if(comp_node->layerGraph->distance(adj_node->layerGraph) < glob_vars::adj_comp_dist){//指在连接过程中可能会有相互影响的components
                comp_node->adj_components.push_back(adj_node);
            }
        }
    }

    /* calculate the center node */
    this->center_node = this->getCenterNode();
    this->writeMajorComponents();
    printf("Connection Relationship:\n");
    this->printConnectionRelation();
    this->printComponentContactingNum();
}

/****
 * 得到拓扑意义的中心节点，具体意义是
 *  一个节点的value是它到其他所有节点的***物理距离***乘以其他节点的hole number
 *  然后value最小的节点就是中心节点
 */
Component* ConstructionGraph::getCenterNode(){
    double* node_value = new double[this->components.size()]{0.0};
    for(int i = 0; i < this->components.size(); i++){
        LayerGraph* layer_i = this->components.at(i)->layerGraph;
        for(int j = 0; j < this->components.size(); j++){
            LayerGraph* layer_j = this->components.at(j)->layerGraph;
            node_value[i] += layer_i->distance(layer_j) * layer_j->hole_num();
        }
    }

    int index_low_value = -1;
    double lowest_value = 99999999.0;
    for(int i = 0; i < this->components.size(); i++){
        printf("%lf\t",node_value[i]);
        if(node_value[i] < lowest_value){
            index_low_value = i;
            lowest_value = node_value[i];
        }
    }
    printf("\n");

    delete[] node_value;

    return this->components.at(index_low_value);
}

void ConstructionGraph::initAffectedHoles(){
    for(auto& comp : components){
        for(auto& h : comp->beamLayout->holes){
            Vector3d current_normal;
            switch( comp->layerGraph->getCurrentPlaneFlag() ){
                case Constants::PLANE_XY :{ current_normal = Vector3d(0,0,1); break; }
                case Constants::PLANE_XZ :{ current_normal = Vector3d(0,1,0); break; }
                case Constants::PLANE_YZ :{ current_normal = Vector3d(1,0,0); break; }
                default: printf("Unorthorgnal Normal!!!\n");
            }
            for(auto& other_comp : components){
                if( other_comp == comp ) continue;

                for(auto& other_h : other_comp->beamLayout->holes ){
                    for(int i = - BeamLayout::MAX_LAYER ; i < BeamLayout::MAX_LAYER; i++ ){
                        if ( (h->position() + i * current_normal - other_h->position()).norm() < 0.9 ){
                            comp->beamLayout->affected_hole[h][i] = other_h;
                        }
                    }
                }
            }
        }
    }

    for(auto& comp : components){
        for(auto& new_b : comp->beamLayout->beams){
            for(auto& h : new_b->template_beam->covered_holes){
                if( comp->beamLayout->affected_hole[h][new_b->layer_number] != NULL ){ // the current beam affect other component
                    auto& outer_hole = comp->beamLayout->affected_hole[h][new_b->layer_number];
                    outer_hole->belonged_beamLayout->outerBeamsOnHole[outer_hole].insert(new_b);
                    if(outer_hole->symmtry_hole != NULL){
                        outer_hole->belonged_beamLayout->outerBeamsOnHole[outer_hole->symmtry_hole].insert(new_b);
                    }
                }
            }
        }
    }
}

void ConstructionGraph::genComponentCandidates3D_baseline()//using a 3D SA to generate the whole result
{
    /***** Initalize all components *****/
    for(auto& component : components){
        component->SAInit();

        set<UnitEdge*, UnitEdge::UnitEdgeIDComp > u_edges(component->layerGraph->unit_edges.begin(),
                                  component->layerGraph->unit_edges.end());

        DataWriter::writeUnitEdges( u_edges,
                                    glob_vars::guidingGraph->m_holes,
                                    std::to_string(component->_id) , true);
    }


    /***** Start SA *****/
    double T = 2000.0;
    double T_min = 0.001;
    double r = 0.9999;
    DataWriter::writeBeamInstances(*this, "_3Dinit");
    double curr_score = Grader::overall_score_3D(*this);
//    DataWriter::writeBeamInstances(*this, "_3Dinit_" + std::to_string(curr_score));
    OperationBeams oprBeams;

    vector<Component*> rand_comp_picker;// All components weighted by the number of possible beams
    for(auto& c : components){
        for(int i = 0; i < c->layerGraph->all_possi_beams.size(); i++){
            rand_comp_picker.push_back(c);
        }
    }


    printf("cooling rate: %lf\n",r);
    cin.get();

    int count = 0;

    while(T > T_min){
        ////////// Pick a random component
        Component* oper_comp = rand_comp_picker[intRandom(0, rand_comp_picker.size()-1)];
        oper_comp->doSimpleOperation_traditionSA(oprBeams);

        if(count % 1000 == 0)
            DataWriter::writeBeamInstances(*this, "_3Dintermediate_" + std::to_string(count));

        double score = Grader::overall_score_3D(*this);

        double dt = curr_score - score;
        if(dt>=0){//说明这个操作是下降的
            //accept this operation, do nothing
            curr_score = score;
        }else{
            printf("current probability:%lf \t T:%lf \n",exp((dt)/T),T);
            if(exp((dt)/T) > cRandom(0,1)){
                //accept this operation
                deleteElements(oprBeams.oldBeams);
                curr_score = score;
            }else{
                oper_comp->beamLayout->undoOperation(oprBeams);

                auto temp = oprBeams.oldBeams;
                oprBeams.oldBeams = oprBeams.newBeams;
                oprBeams.newBeams = temp;
                oper_comp->gradeRecorder.updateAllInfo( *oper_comp->beamLayout, oprBeams );
                oprBeams.newBeams = oprBeams.oldBeams;
                oprBeams.oldBeams = temp;

                deleteElements(oprBeams.newBeams);
            }
        }
        T = r * T ; //anealing
        printf("current score : %lf \t count:%d\n", curr_score ,count++);
    }
}

void ConstructionGraph::genComponentCandidates3D_weighted_picking(){

    /***** Initalize all components *****/
    for(auto& component : components){
        component->SAInit();

        set<UnitEdge*, UnitEdge::UnitEdgeIDComp > u_edges(component->layerGraph->unit_edges.begin(),
                               component->layerGraph->unit_edges.end());

        DataWriter::writeUnitEdges( u_edges,
                                    glob_vars::guidingGraph->m_holes,
                                    std::to_string(component->_id) , true);
    }


    /***** Start SA *****/
    double T = 1800.0;
    double T_min = 0.001;
    double r = 0.9999;
    DataWriter::writeBeamInstances(*this, "_3Dinit");
    double curr_score = Grader::overall_score_3D(*this);
//    DataWriter::writeBeamInstances(*this, "_3Dinit_" + std::to_string(curr_score));
    OperationBeams oprBeams;

    vector<Component*> rand_comp_picker;// All components weighted by the number of possible beams
    for(auto& c : components){
        for(int i = 0; i < c->layerGraph->all_possi_beams.size(); i++){
            rand_comp_picker.push_back(c);
        }
    }

    printf("cooling rate: %lf\n",r);
    int count = 0;

    while(T > T_min){
        if(count % 10000 == 0)
            DataWriter::writeBeamInstances(*this, "_3Dintermediate_" + std::to_string(count));

        ////////// Pick a random component
        Component* oper_comp = rand_comp_picker[intRandom(0, rand_comp_picker.size()-1)];

        oper_comp->doSimpleOperation_traditionSA(oprBeams);

        double score = Grader::overall_score_3D(*this);

        double dt = curr_score - score;
        if(dt>=0){//说明这个操作是下降的
            //accept this operation, do nothing
            curr_score = score;
        }else{
//            printf("current probability:%lf \t T:%lf \n",exp((dt)/T),T);
            if(exp((dt)/T) > cRandom(0,1)){
                //accept this operation
                deleteElements(oprBeams.oldBeams);
                curr_score = score;
            }else{
                oper_comp->beamLayout->undoOperation(oprBeams);

                auto temp = oprBeams.oldBeams;
                oprBeams.oldBeams = oprBeams.newBeams;
                oprBeams.newBeams = temp;
                oper_comp->gradeRecorder.updateAllInfo( *oper_comp->beamLayout, oprBeams );
                oprBeams.newBeams = oprBeams.oldBeams;
                oprBeams.oldBeams = temp;

                deleteElements(oprBeams.newBeams);
            }
        }
        T = r * T ; //anealing
        if(count % 1000 == 0)
            printf("current score : %lf \t count:%d\n", curr_score ,count++);
        count++;
    }

    /*** filling gaps ***/
    for(auto& c : components)
        c->beamLayout->fillGap();

    /***** Store Results *****//* record the generation data */
    printf("Final Score!!\n");
    Grader::overall_score_3D( *this ,true, true );
    DataWriter::writeBeamInstances(*this, "_3DBeamsFinal_" + std::to_string(curr_score));


    cin.get();
}

// always keep no collision among all components during SA
void ConstructionGraph::genComponentCandidates3D_weighted_picking_non_collision()
{

    /***** Initalize all components *****/
    for(auto& component : components){
        component->SAInit();

        set<UnitEdge*, UnitEdge::UnitEdgeIDComp> u_edges(component->layerGraph->unit_edges.begin(),
                                                         component->layerGraph->unit_edges.end());

        DataWriter::writeUnitEdges( u_edges,
                                    glob_vars::guidingGraph->m_holes,
                                    std::to_string(component->_id) , true);
    }

    DataWriter::writeBeamInstances(*this, "_3Dinit");

    /***** Initialize affect holes info *****/
    initAffectedHoles();

    /***** Testing     *****/
    for(auto& comp : components){
        for(auto& h : comp->beamLayout->holes){
            for(int i = - BeamLayout::MAX_LAYER ; i < BeamLayout::MAX_LAYER; i++ ){
                if( comp->beamLayout->affected_hole[h][i] != NULL ){
                    printf("comp: %d hole: %d affect_layer: %d affect_hole: %d affect_comp:%d \n",
                           comp->beamLayout->holes.size(), h->_id, i, comp->beamLayout->affected_hole[h][i]->_id, comp->beamLayout->affected_hole[h][i]->belonged_beamLayout->holes.size() );
                }
            }
        }
    }

    std::cout << "beamLayout: " << components.back()->beamLayout << "\n";
    for(auto& h : components.back()->beamLayout->holes){
        printf("hole %d: ", h->_id );
        if(components.back()->beamLayout->outerBeamsOnHole[h].size() == 0){
            printf("0");
        }else{
            for(auto& b : components.back()->beamLayout->outerBeamsOnHole[h]){
                std::cout<<b << ":" << b->layer_number <<" ";
            }
        }
        printf("\n");
    }
    printf("\n");

    /***** End Testing *****/



    /***** Start SA *****/
    double T = 1800.0;
    double T_min = 0.001;
    double r = 0.9999;
    double curr_score = Grader::overall_score_3D(*this, false, true, true);
//    DataWriter::writeBeamInstances(*this, "_3Dinit_" + std::to_string(curr_score));
    OperationBeams oprBeams;

    vector<Component*> rand_comp_picker;// All components weighted by the number of possible beams
    for(auto& c : components){
        for(int i = 0; i < c->layerGraph->all_possi_beams.size(); i++){
            rand_comp_picker.push_back(c);
        }
    }

    printf("cooling rate: %lf\n",r);
    int count = 0;

    while(T > T_min){
        if(count % 10000 == 0){
//            BeamLayoutConnectorGenerator connectorGenerator(this);
//            connectorGenerator.genComponentConnectors();
            DataWriter::writeBeamInstances(*this, "_3Dintermediate_" + std::to_string(count));
            Grader::overall_score_3D(*this, false, true, true);
        }

        ////////// Pick a random component
        Component* oper_comp = rand_comp_picker[intRandom(0, rand_comp_picker.size()-1)];

        oper_comp->doSimpleOperation_nonCollision(oprBeams);

        double score = Grader::overall_score_3D(*this, false, false);

        double dt = curr_score - score;
        if(dt>=0){//说明这个操作是下降的
            //accept this operation, do nothing
            curr_score = score;
        }else{
//            printf("current probability:%lf \t T:%lf \n",exp((dt)/T),T);
            if(exp((dt)/T) > cRandom(0,1)){
                //accept this operation
                deleteElements(oprBeams.oldBeams);
                curr_score = score;
            }else{

                oper_comp->beamLayout->undoOperation(oprBeams);

                auto temp = oprBeams.oldBeams;
                oprBeams.oldBeams = oprBeams.newBeams;
                oprBeams.newBeams = temp;

                oper_comp->beamLayout->updateOuterBeams(oprBeams);

                ////Self symmetry condition
                if( oper_comp->layerGraph->sketchGroup->isSelfSymmetry() ){
                    oper_comp->beamLayout->updateSelfSymmetryInfo(oprBeams);
                }

                oper_comp->gradeRecorder.updateAllInfo( *oper_comp->beamLayout, oprBeams );
                oprBeams.newBeams = oprBeams.oldBeams;
                oprBeams.oldBeams = temp;

                deleteElements(oprBeams.newBeams);
            }
        }
        T = r * T ; //anealing
        if(count % 1000 == 0)
            printf("current score : %lf \t count:%d\n", curr_score ,count++);
        count++;
    }

    /*** filling gaps ***/
    for(auto& c : components)
        c->beamLayout->fillGap();


    /***** Store Results *****//* record the generation data */
    printf("Final Score!!\n");
    Grader::overall_score_3D( *this ,true, true , true);
    DataWriter::writeBeamInstances(*this, "_3DBeamsFinal_" + std::to_string(curr_score));

}

void ConstructionGraph::genComponentCandidates3D_weighted_picking_non_collision_withConnector(){

    glob_vars::config_file << "\n\n\n------------- Main Simulated Annealing ----------------" << endl;

    /***** Initialize affect holes info *****/
    initAffectedHoles();

    /***** Testing  *****/
//    for(auto& comp : components){
//        for(auto& h : comp->beamLayout->holes){
//            for(int i = - BeamLayout::MAX_LAYER ; i < BeamLayout::MAX_LAYER; i++ ){
//                if( comp->beamLayout->affected_hole[h][i] != NULL ){
//                    printf("comp: %d hole: %d affect_layer: %d affect_hole: %d affect_comp:%d \n",
//                           comp->beamLayout->holes.size(), h->_id, i, comp->beamLayout->affected_hole[h][i]->_id, comp->beamLayout->affected_hole[h][i]->belonged_beamLayout->holes.size() );
//                }
//            }
//        }
//    }

//    std::cout << "beamLayout: " << components.back()->beamLayout << "\n";
//    for(auto& h : components.back()->beamLayout->holes){
//        printf("hole %d: ", h->_id );
//        if(components.back()->beamLayout->outerBeamsOnHole[h].size() == 0){
//            printf("0");
//        }else{
//            for(auto& b : components.back()->beamLayout->outerBeamsOnHole[h]){
//                std::cout<<b << ":" << b->layer_number <<" ";
//            }
//        }
//        printf("\n");
//    }
//    printf("\n");

    /***** End Testing *****/

    std::ostringstream sa_statisctis;


    /***** Start SA *****/
    double T = 1800.0;
    double T_conn_added = 20;
    double T_small = 0.005; //ending point of normal non-connector search
    double T_min = 0.0001;  //final ending point
    double r = glob_vars::current_model.cool_rate;
    double curr_score = Grader::overall_score_3D(*this, false, true, true);
    DataWriter::writeBeamInstances(*this, "_3Dinit_" + std::to_string(curr_score));

    glob_vars::config_file << "Start temperature: " << T << endl;
    glob_vars::config_file << "Connector start temperature: " << T_conn_added << endl;
    glob_vars::config_file << "Annealing step temperature: " << T_small << endl;
    glob_vars::config_file << "Cooling rate: " << r << endl;


    OperationBeams oprBeams;

    vector<Component*> rand_comp_picker;// All components weighted by the number of possible beams
    for(auto& c : components){
        if(glob_vars::current_model.name == "normalbow" && c->_id == 2)
            continue;

        for(int i = 0; i < c->layerGraph->all_possi_beams.size(); i++){
            rand_comp_picker.push_back(c);
        }
    }
    int unConnNum = 0;

    int count = 0;

    vector<Connector*> curr_conns;

    clock_t start = clock();

    while(T > T_small){
        if(count % 1000 == 0){
#ifdef OUTPUT_ANIMATION
            if(T < T_conn_added) {
                BeamLayoutConnectorGenerator temp_generator( this );
                unConnNum = temp_generator.genConnectorsCoverUnCoveredEdges().size();
                temp_generator.genStraightConnectors();
                curr_conns = vector<Connector*> ( temp_generator.connectors.begin(), temp_generator.connectors.end() );
                DataWriter::writeBeamInstances(*this, temp_generator, "_3Dintermediate_" + std::to_string(count));
                DataWriter::writeBeamHoles(temp_generator, "_3Dintermediate_" + std::to_string(count));
            }else{
                DataWriter::writeBeamInstances(*this, "_3Dintermediate_" + std::to_string(count));
            }
#endif
            Grader::overall_score_3D(*this, false, true, true);
        }

        ////////// Pick a random component
        Component* oper_comp = NULL;

        oper_comp = rand_comp_picker[intRandom(0, rand_comp_picker.size()-1)];
        oper_comp->doSimpleOperation_nonCollision(oprBeams);

        double score = Grader::overall_score_3D(*this, false, fabs(glob_vars::current_model.generating_weight[6])>0.02, false);
                                                                   //automatically adding collision evaluation according to weight
        double dt = curr_score - score;

        if(dt>=0){//说明这个操作是下降的
            //accept this operation, do nothing
            curr_score = score;
        }else{
//            printf("current probability:%lf \t T:%lf \n",exp((dt)/T),T);
            if(exp((dt)/T) > cRandom(0,1)){
                //accept this operation
//                deleteElements(oprBeams.oldBeams);
                curr_score = score;
            }else{
                oper_comp->beamLayout->undoOperation(oprBeams);

                auto temp = oprBeams.oldBeams;
                oprBeams.oldBeams = oprBeams.newBeams;
                oprBeams.newBeams = temp;

                //////debugging....
//                this->beamLayoutConnectorGenerator->updateStatus(oprBeams);
//                BeamLayoutConnectorGenerator temp_generator( this );
//                printf("real_beamholes_size:%d \t curr_beamholes_size:%d \n", temp_generator.beamholes.size(), this->beamLayoutConnectorGenerator->beamholes.size());
//                printf("real_collision_size:%d \t curr_collision_size:%d \n", temp_generator.getCollisionNum(), this->beamLayoutConnectorGenerator->getCollisionNum());

                oper_comp->beamLayout->updateOuterBeams(oprBeams);

                ////Self symmetry condition
                if( oper_comp->layerGraph->sketchGroup->isSelfSymmetry() ){
                    oper_comp->beamLayout->updateSelfSymmetryInfo(oprBeams);
                }

                oper_comp->gradeRecorder.updateAllInfo( *oper_comp->beamLayout, oprBeams );
                oprBeams.newBeams = oprBeams.oldBeams;
                oprBeams.oldBeams = temp;

//                deleteElements(oprBeams.newBeams);
            }
        }

        if(count % 1000 == 0)
            printf("current score : %lf \t count:%d \t T:%lf \n", curr_score , count++, T);

#ifdef OUTPUT_ANIMATION
//        if(count % 200 == 0){
//            double output_score = curr_score;
//
//            if( T < T_conn_added ){
//                BeamLayoutConnectorGenerator temp_generator( this );
//                unConnNum = temp_generator.genConnectorsCoverUnCoveredEdges().size();
//                temp_generator.genStraightConnectors();
//                curr_conns = vector<Connector*> ( temp_generator.connectors.begin(), temp_generator.connectors.end() );
//                output_score += glob_vars::current_model.generating_weight[7] * unConnNum +
//                                glob_vars::current_model.generating_weight[8] * Grader::Nonpin_head_ratio(curr_conns);
//            }
//            sa_statisctis << output_score << "\n";
//        }
#endif

        T = r * T ; //anealing
        count++;
    }

    glob_vars::config_file << "Major SA final time: \t" << (double)(clock()-start)/CLOCKS_PER_SEC << endl;


    /***** Store Results *****//* record the generation data */
    printf("Final Score: \n");
    Grader::overall_score_3D( *this ,true, true , true );
    DataWriter::writeBeamInstances(*this, "_3DBeamsFinal_" + std::to_string(curr_score));
#ifdef OUTPUT_ANIMATION
    DataWriter::writeDecomposeSAStatistics(sa_statisctis.str(), "majorSA_Phase1");
#endif

}

void ConstructionGraph::genComponentCandidates3D_weighted_picking_non_collision_withConnector_Phase2() //enroll connector into consideration
{
    /*------------------------------------------ Start SA Phase two ---------------------------------------------*/
    std::ostringstream sa_statisctis;

    int count = 0;
    double curr_score = Grader::overall_score_3D(*this, false, false, false);

    BeamLayoutConnectorGenerator connectorGenerator( this );
    vector<UnitEdgeGroup*> unCoveredGroups = connectorGenerator.genConnectorsCoverUnCoveredEdges();
    connectorGenerator.genStraightConnectors();
    vector<Connector*> curr_conns = vector<Connector*>(connectorGenerator.connectors.begin(),connectorGenerator.connectors.end());

    curr_score += glob_vars::current_model.generating_weight[7] * unCoveredGroups.size() +
                  glob_vars::current_model.generating_weight[8] * Grader::Nonpin_head_ratio(curr_conns);


    vector<Component*> rand_comp_picker;// All components weighted by the number of possible beams
    for(auto& c : components){
        for(int i = 0; i < c->layerGraph->all_possi_beams.size(); i++){
            rand_comp_picker.push_back(c);
        }
    }


    double T = 0.005; //ending point of normal non-connector search
    double T_min = 0.0049;  //final ending point
    double r = 0.9999;

    OperationBeams oprBeams;

    while(T > T_min){

        ////////// Pick a random component
        Component* oper_comp = NULL;

        vector<UnitEdge*> relatedEdges;

        for(auto& g : unCoveredGroups){
            for(auto& h : g->related_layerholes){
                if( h->belonged_beamLayout != NULL ){
                    for(auto& u : h->unit_edges){
                        if(u->belonged_beamLayout != NULL)
                            relatedEdges.push_back(u);
                    }
                }
            }
        }

        if(relatedEdges.empty()){
            printf("No contain-beams holes!!\n");
            BeamLayoutConnectorGenerator connectorGenerator( this );
            unCoveredGroups = connectorGenerator.genConnectorsCoverUnCoveredEdges();
            printf("unGroups:%d\n", unCoveredGroups.size() );
            if( !unCoveredGroups.empty() )
                printf("Problem found\n");
            oper_comp = rand_comp_picker[intRandom(0, rand_comp_picker.size()-1)];
            oper_comp->doSimpleOperation_nonCollision(oprBeams);

        }else{
            UnitEdge* rand_Edge = relatedEdges[intRandom(0, relatedEdges.size()-1)];
            oper_comp = rand_Edge->belonged_beamLayout->belonged_component;
            oper_comp->doSimpleOperation_unCoveredEdges(oprBeams, rand_Edge);
        }

        double score = Grader::overall_score_3D(*this, false, false, true);

        BeamLayoutConnectorGenerator temp_generator( this );
        unCoveredGroups = temp_generator.genConnectorsCoverUnCoveredEdges();
        temp_generator.genStraightConnectors();
        curr_conns = vector<Connector*>(temp_generator.connectors.begin(), temp_generator.connectors.end());
        score = score + glob_vars::current_model.generating_weight[7] * unCoveredGroups.size() +
                        glob_vars::current_model.generating_weight[8] * Grader::Nonpin_head_ratio(curr_conns);

#ifdef OUTPUT_ANIMATION
        DataWriter::writeBeamInstances(*this, temp_generator, "_3Dintermediate2_" + std::to_string(count));
#endif

        double dt = curr_score - score;


        if(dt>=0){//说明这个操作是下降的
            //accept this operation, do nothing
            curr_score = score;

        }else{
//            printf("current probability:%lf \t T:%lf \n",exp((dt)/T),T);
            if(exp((dt)/T) > cRandom(0,1)){
                //accept this operation
                deleteElements(oprBeams.oldBeams);
                curr_score = score;
            }else{

                oper_comp->beamLayout->undoOperation(oprBeams);

                auto temp = oprBeams.oldBeams;
                oprBeams.oldBeams = oprBeams.newBeams;
                oprBeams.newBeams = temp;

                oper_comp->beamLayout->updateOuterBeams(oprBeams);

                ////Self symmetry condition
                if( oper_comp->layerGraph->sketchGroup->isSelfSymmetry() ){
                    oper_comp->beamLayout->updateSelfSymmetryInfo(oprBeams);
                }

                oper_comp->gradeRecorder.updateAllInfo( *oper_comp->beamLayout, oprBeams );
                oprBeams.newBeams = oprBeams.oldBeams;
                oprBeams.oldBeams = temp;

                deleteElements(oprBeams.newBeams);
            }
        }
        T = r * T ; //anealing
        printf("unconnectness:%d current score : %lf \t count:%d \t T:%lf \n", unCoveredGroups.size(), curr_score , count, T);

#ifdef OUTPUT_ANIMATION
        sa_statisctis << curr_score << "\n";
#endif

        count++;
    }

#ifdef OUTPUT_ANIMATION
    DataWriter::writeDecomposeSAStatistics(sa_statisctis.str(), "majorSA_Phase2");
#endif

}

void ConstructionGraph::tryTwoOrientForMinorComponents(){

    std::ostringstream sa_statisctis;

    /**** Collecting all minor components ****/
    vector<Component*> minor_components;
    for(auto& comp : components){
        if( !comp->layerGraph->isUniqueOrient() && !comp->beamLayout->beams.empty())
            minor_components.push_back( comp );
    }

    int count = 0;
    for( auto& c : minor_components ){

        int best_orient = 0;
        double lowest_score = 1e9;
        for(int orient_idx : {0, 1}){
            c->layerGraph->orientation_index = orient_idx;

            BeamLayoutConnectorGenerator connectorGenerator(this);
            int unConnNum = connectorGenerator.genConnectorsCoverUnCoveredEdges().size();
            connectorGenerator.genStraightConnectors();

            vector<Connector*> current_connectors(connectorGenerator.connectors.begin(), connectorGenerator.connectors.end());
            double current_score = Grader::Nonpin_head_ratio(current_connectors);



            double output_score = Grader::overall_score_3D(*this, false, false, false) +
                                  glob_vars::current_model.generating_weight[7] * unConnNum   +
                                  glob_vars::current_model.generating_weight[8] * Grader::Nonpin_head_ratio(current_connectors);//The uncovered unit edges should be zero here


#ifdef OUTPUT_ANIMATION
            c->layerGraph->updateBeamOrientation();
            DataWriter::writeBeamInstances(*this, connectorGenerator, "_3Dintermediate3_" + std::to_string(count++));
            sa_statisctis << output_score << "\n";
#endif

            if(current_score < lowest_score){
                best_orient = orient_idx;
                lowest_score = current_score;
            }
        }

        c->layerGraph->orientation_index = best_orient;
        c->layerGraph->updateBeamOrientation();

    }

#ifdef OUTPUT_ANIMATION
    DataWriter::writeDecomposeSAStatistics(sa_statisctis.str(), "majorSA_Phase3");
#endif

}


void ConstructionGraph::deleteCollideElements(){

    vector< tuple<Vector3d, int, vector<BeamInstance*> , vector<Component*> > > total_holes; //list of mapping from point to the number of same-position points

    for(auto& comp : components){
        for(auto& b : comp->beamLayout->beams){
            BeamTemplate *temp_b = b->getBeam();
            auto beam_holes = temp_b->getCurrentComponentHoles();
            for(auto& hole : beam_holes){
                ///// search for all existing holes
                auto find_iter = std::find_if(total_holes.begin(), total_holes.end(), [hole](tuple<Vector3d, int,
                                                                                             vector<BeamInstance*> , vector<Component*> >& exist_hole){
                    return  (hole - std::get<0>(exist_hole)).norm() < 0.9; //collision if distance between two hole center is less than 1.0
                });

                if(find_iter == total_holes.end()){ // not exist yet
                    vector<BeamInstance*> b_vec = {b};
                    vector<Component*>    l_vec = {comp};
                    total_holes.push_back( std::make_tuple(hole, 1, b_vec , l_vec ) );
                } else{ //already exist
                    std::get<1>(*find_iter) += 1;
                    std::get<2>(*find_iter).push_back(b);
                    std::get<3>(*find_iter).push_back(comp);
                }
            }
        }
    }

    //// Remove holes without collison
    total_holes.erase(std::remove_if(total_holes.begin(), total_holes.end(), []( tuple<Vector3d, int, vector<BeamInstance*> , vector<Component*> >& exist_hole){
                                                                return std::get<1>(exist_hole) < 2;
                                                            }
                                                          ),
                      total_holes.end()
    );

    //TODO: above tested, but haven't finished

}


void ConstructionGraph::genComponentCanndidate3D_weighted_picking_adptiveSchedul(){

    /***** Initalize all components *****/
    for(auto& component : components){
        component->SAInit();

        set<UnitEdge*, UnitEdge::UnitEdgeIDComp> u_edges(component->layerGraph->unit_edges.begin(),
                                                         component->layerGraph->unit_edges.end());

        DataWriter::writeUnitEdges( u_edges,
                                    glob_vars::guidingGraph->m_holes,
                                    std::to_string(component->_id) , true);
    }


    /***** Start SA *****/
    double T = 1800.0;
    double T_min = 0.001;
    DataWriter::writeBeamInstances(*this, "_3Dinit");

    double curr_score = Grader::overall_score_3D(*this);

    //Parameters for adaptive SA
    double deviation = curr_score * curr_score;
    long   solution_num = 1;
    const double delta = 0.008;

    OperationBeams oprBeams;

    vector<Component*> rand_comp_picker;// All components weighted by the number of possible beams
    for(auto& c : components){
        for(int i = 0; i < c->layerGraph->all_possi_beams.size(); i++){
            rand_comp_picker.push_back(c);
        }
    }

    int count = 0;

    while(T > T_min){
        if(count % 10000 == 0)
            DataWriter::writeBeamInstances(*this, "_3Dintermediate_" + std::to_string(count));

        ////////// Pick a random component
        Component* oper_comp = rand_comp_picker[intRandom(0, rand_comp_picker.size()-1)];

        oper_comp->doSimpleOperation_traditionSA(oprBeams);

        double score = Grader::overall_score_3D(*this);

        //// update solution deviation
        deviation = ((deviation * solution_num) + (score * score)) / (solution_num + 1);
        solution_num ++;

        double dt = curr_score - score;
//        printf("current probability:%lf \t T:%lf \n",exp((dt)/T),T);
        if(dt>=0){//说明这个操作是下降的
            //accept this operation, do nothing
            curr_score = score;
        }else{
            if(exp((dt)/T) > cRandom(0,1)){
                //accept this operation
                deleteElements(oprBeams.oldBeams);
                curr_score = score;
            }else{
                oper_comp->beamLayout->undoOperation(oprBeams);

                auto temp = oprBeams.oldBeams;
                oprBeams.oldBeams = oprBeams.newBeams;
                oprBeams.newBeams = temp;
                oper_comp->gradeRecorder.updateAllInfo( *oper_comp->beamLayout, oprBeams );
                oprBeams.newBeams = oprBeams.oldBeams;
                oprBeams.oldBeams = temp;

                deleteElements(oprBeams.newBeams);
            }
        }
        T = T / (1 + T * log(1 + delta) / (3 * sqrt(deviation) ) ) ; //adaptive anealing
        if(count % 1000 == 0)
            printf("current score : %lf \t count:%d\n", curr_score ,count++);
    }

    /***** Store Results *****//* record the generation data */
    printf("Final Score!!n");
    Grader::overall_score_3D( *this );
    DataWriter::writeBeamInstances(*this, "_3DFinal_" + std::to_string(curr_score));

    cin.get();
}

vector<vector<Component*>> ConstructionGraph::groupingByAdjacent(set<Component*>& layernodes){
    set<Component*> all_nodes(layernodes.begin(),layernodes.end());
    vector<vector<Component*>> result;
    while(!all_nodes.empty()){
        Component* node = *all_nodes.begin();
        vector<Component*> curr_group = {node};
        for(auto& adj : all_nodes){
            if(std::find(node->adj_components.begin(),node->adj_components.end(),adj) != node->adj_components.end()){
                curr_group.push_back(adj);
            }
        }
        result.push_back(curr_group);
        for(auto& g : curr_group)
            all_nodes.erase(g);
    }

    return result;
}


void ConstructionGraph::printConnectionRelation(){
    for(auto& comp : this->components){
        printf("%d->",comp->_id);
        for(auto& adj_node : comp->adj_components){
            printf(" %d (%d)", adj_node->_id, comp->layerGraph->contactPointsNum(*adj_node->layerGraph));
        }
        printf("\n");
    }
    printf("Center:%d\n",this->center_node->_id);
}

void ConstructionGraph::printComponentContactingNum(){
    for(auto iter = components.begin(); iter != components.end(); iter ++){
        for(auto iter2 = iter+1; iter2 != components.end(); iter2++){
            int conn_num = (*iter)->layerGraph->contactPointsNum(*(*iter2)->layerGraph);
            printf("ID:%d \t -> ID:%d \t contacting number: %d\n",(*iter)->_id,(*iter2)->_id, conn_num );
        }
    }
}

void ConstructionGraph::writeMajorComponents(){
    assert(this->center_node != NULL);
    int color = 12;
    for(auto& component : this->components){
        string center_str = component == this->center_node? "_center" : "";
        string file_path = glob_vars::current_debug_dir + "/" + glob_vars::current_model.name + "_component_"
                           + std::to_string(component->_id) + center_str + ".ldr";
        using std::ofstream;
        ofstream fout;
        fout.open(file_path);
        fout<< "0 component id:" << component->_id << endl;
        fout<<component->layerGraph->getCurrentHolesAsString(component == this->center_node? 75 : color++);
        fout.close();
        printf("File %s created\n",file_path.c_str());
    }
}
