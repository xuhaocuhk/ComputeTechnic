//
// Created by 徐豪 on 2018-10-15.
//
#define _USE_MATH_DEFINES
#include <bricks/BrickFactory.h>
#include <util/Timer.h>
#include "BeamLayoutConnectorGenerator.h"
#include "util/Helper.h"
#include "util/util.h"
#include "configure/global_variables.h"
#include "core/UnitEdgeGroup.h"
#include "hard_code/BoundingBox.h"
#include <cmath>


BeamLayoutConnectorGenerator::BeamLayoutConnectorGenerator(ConstructionGraph* m_constructionGraph){
    this->constructionGraph = m_constructionGraph;
    initBeamHoles();
}

using std::cout;
void BeamLayoutConnectorGenerator::genStraightConnectors(){

    BrickFactory brickFactory;
    vector<Connector> straight_connectors = brickFactory.getAllStraightConnectors();

    for(auto& h : glob_vars::guidingGraph->m_holes){

        vector<int> numbers;
        for(auto& beamhole : beamholesOnLayerHole[h])
        {
            if(beamhole->layer_number_on_beam > 100 || !connectorsOnHole[beamhole].empty() ) continue;
            numbers.push_back(beamhole->layer_number_on_beam);
        }

        if(numbers.size() >= 2 ){
            //// get top and bottom beam hole position
            auto max = std::max_element(numbers.begin(),numbers.end());
            auto min = std::min_element(numbers.begin(),numbers.end());

            Vector3d max_pos = h->position() + (*max) * beamholesOnLayerHole[h].front()->normal;
            Vector3d min_pos = h->position() + (*min) * beamholesOnLayerHole[h].front()->normal;

            //// Check close_hole collision
            if( std::find_if( h->close_holes.begin(), h->close_holes.end(), [this, min, max](LayerHole* close_hole){
                for(auto& c_hole : this->beamholesOnLayerHole[close_hole]){
                    if ( c_hole->layer_number_on_beam >= (*min) && c_hole->layer_number_on_beam <= (*max))
                        return true;
                }
                return false;
            }) != h->close_holes.end()){ continue; }


            //// Pick corresponding connector
            Connector straight_conn;
            if( (*max) - (*min) > 6 ){
                printf("Layer too thick!! layer num:%d \n", (*max) - (*min) );
                cout<< "hole: " << h->position() << endl;
                cin.get();
                continue;
            }else{
                if((*max) - (*min) - 1 < 0){
                    printf("problem here!!!\n");
                    h->toLdrawFormat();
                    for(auto& bhhh : beamholesOnLayerHole[h])
                        bhhh->toLDrawFormat(8);
                }else{
                    straight_conn = straight_connectors[(*max) - (*min) - 1];
                }
            }

            //// get two end insertable point of the connector
//            printf("%s\n", straight_conn.name.c_str());
            auto insert_pins = straight_conn.getCurrentInsertablePins();
            Vector3d end1_pos = insert_pins.front().pos; //end1_pos -> max_pos
            Vector3d end2_pos = insert_pins.back().pos;  //end2_pos -> min_pos

            Vector3d conn_vec = end2_pos - end1_pos; conn_vec.normalize();
            Vector3d hole_vec = min_pos  - max_pos ; hole_vec.normalize();
            Vector3d rotation_axis = conn_vec.cross(hole_vec);
            rotation_axis = rotation_axis.norm() == 0 ? Vector3d(0,0,1) : rotation_axis; // when two vectors are co-linear, choose any random orthorgonal axis as rotation axis
            Matrix3d rot_mat = Eigen::AngleAxisd(acos(conn_vec.dot(hole_vec)/(conn_vec.norm()*hole_vec.norm())), rotation_axis/rotation_axis.norm()).matrix();

            straight_conn.transform(rot_mat);
            insert_pins = straight_conn.getCurrentInsertablePins();
            end1_pos = insert_pins.front().pos; //end1_pos -> max_pos

            straight_conn.translate( max_pos - end1_pos );

            straight_conn.connected_holes = beamholesOnLayerHole[h];

            Connector* pointer_connector = new Connector(straight_conn);
            connectors.insert(pointer_connector);
            straight_conns.insert(pointer_connector);
            for(auto& beamhole : beamholesOnLayerHole[h]){
                connectorsOnHole[beamhole].push_back(pointer_connector);
            }
        }
    }

}

vector<Connector> BeamLayoutConnectorGenerator::findAllPossibleConnector(set<BeamHole*>& comp_hole_1, set<BeamHole*>& comp_hole_2) {
    vector<Connector> all_possible_conns;

    BrickFactory brickFactory;
    vector<Connector> connectorss = brickFactory.getAllConnectors();

    for(auto& hole : comp_hole_1){
        for(auto connector : connectorss)
        {
            Pin first_pin = connector.getCurrentInsertablePins().front();
            for( int k : {0, 1} ){//try both sides of a layer
                ///Insert the first pin into corresponding hole
                Matrix3d base;
                /* rotate to align the first pin with beam hole  */
                Vector3d pin_orient = first_pin.orientation;
                Vector3d h_normal = hole->normal;
                Vector3d rotation_axis = pin_orient.cross(h_normal);
                if(rotation_axis.norm() == 0)//colinear, already in the right orientation
                    base = Matrix3d::Identity(3,3);
                else{
                    base = Eigen::AngleAxisd(acos(pin_orient.dot(h_normal)/(pin_orient.norm()*h_normal.norm())),
                                             rotation_axis/rotation_axis.norm()).matrix();
                }

                if(k == 0){
                    //doing nothing
                } else {
                    /* reflect to another side */
                    Vector3d perpendicular_vec = (base * first_pin.orientation).cross( Vector3d(1,0,0) );
                    if(perpendicular_vec.norm() == 0)
                    {
                        perpendicular_vec = Vector3d(0,1,0);
                    }

                    base = Eigen::AngleAxisd(M_PI, perpendicular_vec/perpendicular_vec.norm() ).matrix() * base;
                }

                //update current first_pin
                first_pin = connector.getCurrentInsertablePins().front();

                for(int r : {0 , 1 , 2 , 3}){
                    Matrix3d rot_mat = Eigen::AngleAxisd(M_PI/2 * r, hole->normal).matrix();
                    Matrix3d transform_mat = rot_mat * base;

                    //apply transformation and translation
                    connector.transform(transform_mat);
                    connector.translate(hole->pos - connector.getCurrentInsertablePins().front().pos );

                    //Checking point!!
                    if( this->allPinsAvailable(connector, comp_hole_1, comp_hole_2) &&
                        this->noCollision_non_pin_components(connector, comp_hole_1, comp_hole_2 ) &&
                        this->bridgeTwoComponents(connector, comp_hole_1, comp_hole_2) )
                    {
                        if(!contains(all_possible_conns, connector))
                            all_possible_conns.push_back(connector);
                    }

                    //reset the transformation
                    connector.resetTransformation();
                }
            }
        }
    }

    return all_possible_conns;
}

set<UnitEdge*> BeamLayoutConnectorGenerator::getAllUnCoveredEdges(){

    set<UnitEdge*> uncovered_edges;
    for(auto& ue : glob_vars::guidingGraph->u_edges){
        auto symm_uedges = ue->getAllSymmetryUnitEdges();

        auto iter = std::find_if(symm_uedges.begin(), symm_uedges.end(), [](UnitEdge* symm_ue){
            return symm_ue->belonged_beamLayout != NULL && !symm_ue->belonged_beamLayout->beamsOnEdge[symm_ue].empty();
        });

        if( iter == symm_uedges.end() ){ //all symmetry edges are not covered
            uncovered_edges.insert(ue);
        }
    }
    return uncovered_edges;

}

vector<UnitEdgeGroup*> BeamLayoutConnectorGenerator::groupAllUnCoveredEdges(set<UnitEdge*>& ucEdges){
    vector< UnitEdgeGroup* > uc_groups;

    while( !ucEdges.empty() ){
        UnitEdge* unitEdge = *ucEdges.begin();
        set<UnitEdge*, UnitEdge::UnitEdgeIDComp > group = unitEdge->getAllConnectEdgesInGroup(ucEdges);
        UnitEdgeGroup * uedgeGroup = new UnitEdgeGroup(group);
        uc_groups.push_back( uedgeGroup );

        for(auto& ue : group) ucEdges.erase(ue);
    }

    return uc_groups;
}

void BeamLayoutConnectorGenerator::eraseBeam(BeamInstance* beamInstance){

    for(auto& bh : beamInstance->beamholes){
        beamholes.erase(bh);
        beamholesOnLayerHole[bh->layerHole].erase( std::remove(beamholesOnLayerHole[bh->layerHole].begin(), beamholesOnLayerHole[bh->layerHole].end(), bh),
                                                   beamholesOnLayerHole[bh->layerHole].end() );
        connectorsOnHole.erase(bh);
        if(collisionHoles.find(bh) != collisionHoles.end()){
            for(auto& clbh : collisionHoles[bh]){
                collisionHoles[clbh].erase(bh);
            }
            collisionHoles.erase(bh);
        }
    }
    assert(beamholes.size() == connectorsOnHole.size());
}

void BeamLayoutConnectorGenerator::addBeam(BeamInstance* beamInstance){

    for(auto& real_lh : beamInstance->template_beam->covered_holes){
        auto all_symm_lh = real_lh->getAllSymmetryHoles();

//        printf("------\n");
//        real_lh->toLdrawFormat(5);
//        for(auto& lhhh : all_symm_lh)
//            lhhh->toLdrawFormat(6);

        for(auto lh : all_symm_lh){
            auto beamholes4beam = lh->getBeamHolesOnthisLayerHoleGivenBeam(beamInstance);
            for(auto& new_bh : beamholes4beam){
                ////judge collision
                for(auto& bh : beamholes){
                    if( (new_bh->pos - bh->pos).norm() < 0.9 ){
                        collisionHoles[new_bh].insert(bh);
                        collisionHoles[bh].insert(new_bh);
                    }
                }
                //update data structure
                beamholes.insert(new_bh);
                beamholesOnLayerHole[lh].push_back(new_bh);
                connectorsOnHole.insert(std::pair<BeamHole*, vector<Connector*> >(new_bh, vector<Connector*>() ) );
                new_bh->layerHole = lh;
            }
        }
    }
}

void BeamLayoutConnectorGenerator::updateStatus(OperationBeams& optBeams){

    for(auto& b : optBeams.oldBeams){
        eraseBeam(b);
    }

    for(auto& b : optBeams.newBeams){
        addBeam(b);
    }
}

int BeamLayoutConnectorGenerator::getCollisionNum(){
    int collid = 0;
    for(auto& coll : collisionHoles){
        collid += coll.second.size();
    }
    return collid / 2;
}

vector<Connector> BeamLayoutConnectorGenerator::findAllPossibleConnector(set<BeamHole*>& beamhole_set){
    vector<Connector> all_possible_conns;

    BrickFactory brickFactory;
    vector<Connector> connectorss = brickFactory.getAllConnectors();

    for(auto& hole : beamhole_set){
        for(auto connector : connectorss)
        {
            Pin first_pin = connector.getCurrentInsertablePins().front();
            for( int k : {0, 1} ){//try both sides of a layer
                ///Insert the first pin into corresponding hole
                Matrix3d base;
                /* rotate to align the first pin with beam hole  */
                Vector3d pin_orient = first_pin.orientation;
                Vector3d h_normal = hole->normal;
                Vector3d rotation_axis = pin_orient.cross(h_normal);
                if(rotation_axis.norm() == 0)//colinear, already in the right orientation
                    base = Matrix3d::Identity(3,3);
                else{
                    base = Eigen::AngleAxisd(acos(pin_orient.dot(h_normal)/(pin_orient.norm()*h_normal.norm())),
                                             rotation_axis/rotation_axis.norm()).matrix();
                }

                if(k == 0){
                    //doing nothing
                } else {
                    /* reflect to another side */
                    Vector3d perpendicular_vec = (base * first_pin.orientation).cross( Vector3d(1,0,0) );
                    if(perpendicular_vec.norm() == 0)
                    {
                        perpendicular_vec = Vector3d(0,1,0);
                    }

                    base = Eigen::AngleAxisd(M_PI, perpendicular_vec/perpendicular_vec.norm() ).matrix() * base;
                }

                //update current first_pin
                first_pin = connector.getCurrentInsertablePins().front();

                for(int r : {0 , 1 , 2 , 3}){
                    Matrix3d rot_mat = Eigen::AngleAxisd(M_PI/2 * r, hole->normal).matrix();
                    Matrix3d transform_mat = rot_mat * base;

                    //apply transformation and translation
                    connector.transform(transform_mat);
                    connector.translate(hole->pos - connector.getCurrentInsertablePins().front().pos );

                    //Checking point!!
                    if( this->allPinsAvailable(connector, beamhole_set) &&
                        this->noCollision_non_pin_components(connector, beamhole_set) )
                    {
                        if( !contains(all_possible_conns, connector)  && inBoundingBox(connector))
                            all_possible_conns.push_back(connector);
                    }

                    //reset the transformation
                    connector.resetTransformation();
                }
            }
        }
    }

    return all_possible_conns;
}

vector<UnitEdgeGroup*> BeamLayoutConnectorGenerator::genConnectorsCoverUnCoveredEdges() // return number of unconnected groups
{

    vector<UnitEdgeGroup*> unCoveredGroups;

    /***** Collect all uncovered edges *****/
    set<UnitEdge*> uncovered_edges = getAllUnCoveredEdges();


    /***** Grouping uncovered edges *****/
    this->uncoverd_uedges_groups = groupAllUnCoveredEdges(uncovered_edges);

    ////Establish symmetry relation
    UnitEdgeGroup::establishSymmetryGroupRelation( uncoverd_uedges_groups );


    ////Sorting according to group size
    std::sort(uncoverd_uedges_groups.begin(), uncoverd_uedges_groups.end(), []( UnitEdgeGroup * group1, UnitEdgeGroup * group2 ){
        return group1->u_edges.size() > group2->u_edges.size();
    });

    /****** debugging for grouping result ******/
//    for(auto& group : uncoverd_uedges_groups){
//        for(auto& ue : group->u_edges)
//            printf("%d ", ue->_id);
//        printf("\n");
//    }

    for(auto& group : uncoverd_uedges_groups){

        bool is_generated = group->genConnectorsForThisGroup( *this );
        if(!is_generated)
            unCoveredGroups.push_back(group);

    }
    return unCoveredGroups;
}


void BeamLayoutConnectorGenerator::initBeamHoles(){

    ////Initialize beamholes on layerholes
    for(auto& h : glob_vars::guidingGraph->m_holes){
        beamholesOnLayerHole.insert( std::pair<LayerHole*, vector<BeamHole*>>(h, vector<BeamHole*>()) );
    }

    ////Generate beamholes
    for(auto& h : glob_vars::guidingGraph->m_holes){
        auto beams_on_layerhole = h->getBeamHolesOnThisLayerHole();

        beamholesOnLayerHole.insert( std::pair<LayerHole*, vector<BeamHole*>>
                                      (h, beams_on_layerhole) );

        for(auto& bh_on_h : beams_on_layerhole){
            beamholes.insert(bh_on_h);
            beamholesOnLayerHole[h].push_back(bh_on_h);
            bh_on_h->layerHole = h;
        }
    }

    ////Init Collision pairs
    for(auto& h1 : beamholes){
        for(auto& h2 : beamholes){
            if(h1 == h2) continue;
            if( (h1->pos - h2->pos).norm() < 0.9 ){
                collisionHoles[h1].insert(h2);
                collisionHoles[h2].insert(h1);
            }
        }
    }

    //// Initialize connectorsOnHole
    for(auto& h : beamholes){
        connectorsOnHole.insert(std::pair<BeamHole*, vector<Connector*> >(h, vector<Connector*>() ) );
    }

    //// Initialize uncovered unit edge groups
    set<UnitEdge*> unEdges = getAllUnCoveredEdges();
    this->uncoverd_uedges_groups = groupAllUnCoveredEdges(unEdges);
    for(auto& group : this->uncoverd_uedges_groups){
        for(auto& ue : group->u_edges){
            this->belongedGroup[ue] = group;
        }
    }



}

//Check whether all pins have holes to insert
bool BeamLayoutConnectorGenerator::allPinsAvailable(Connector& connector, set<BeamHole*>& current_holes, set<BeamHole*>& adj_holes ) // holes in current / adjacent component
{
    double POS_THRESHHOLD = 0.1;
    double ORIEN_THRESHHOLD = 0.1;
    vector<BeamHole*> connectHoles;

    vector<Pin> insertable_pins = connector.getCurrentInsertablePins();

    for(int j = 0; j < insertable_pins.size();j++)
    {
        Pin& pin_j = insertable_pins.at(j);
        //检查对Beam Hole 的存在性
        bool exist = false;

        for(auto& beamHole : current_holes){
            if( fabs(fabs(pin_j.orientation.dot(beamHole->normal))-1.0)<ORIEN_THRESHHOLD &&
                (pin_j.pos-beamHole->pos).norm() < POS_THRESHHOLD) //same orientation & same position
            {
                connectHoles.push_back(beamHole);
                exist = true;
            }
        }

        for(auto& beamHole : adj_holes){
            if( fabs(fabs(pin_j.orientation.dot(beamHole->normal))-1.0)<ORIEN_THRESHHOLD &&
                (pin_j.pos-beamHole->pos).norm() < POS_THRESHHOLD) //same orientation & same position
            {
                connectHoles.push_back(beamHole);
                exist = true;
            }
        }

        if(!exist)
            return false;
    }

    connector.connected_holes.clear();
    for(const auto & h : connectHoles){
        connector.connected_holes.push_back(h);
    }
//    if( connector.connected_holes.size() != connector.getCurrentInsertablePins().size() )
//        printf("One insertable pin connect more than one holes!! May contains collision in the model\n ");

    return true;
}

bool BeamLayoutConnectorGenerator::allPinsAvailable(Connector& connector, set<BeamHole*>& current_holes){
    double POS_THRESHHOLD = 0.1;
    double ORIEN_THRESHHOLD = 0.1;
    vector<BeamHole*> connectHoles;

    vector<Pin> insertable_pins = connector.getCurrentInsertablePins();

    for(int j = 0; j < insertable_pins.size();j++)
    {
        Pin& pin_j = insertable_pins.at(j);
        //检查对Beam Hole 的存在性
        bool exist = false;

        for(auto& beamHole : current_holes){
            if( fabs(fabs(pin_j.orientation.dot(beamHole->normal))-1.0) < ORIEN_THRESHHOLD &&
                (pin_j.pos-beamHole->pos).norm() < POS_THRESHHOLD) //same orientation & same position
            {
                connectHoles.push_back(beamHole);
                exist = true;
            }
        }

        if(!exist)
            return false;
    }

    connector.connected_holes.clear();
    for(const auto & h : connectHoles){
        connector.connected_holes.push_back(h);
    }
//    if( connector.connected_holes.size() != connector.getCurrentInsertablePins().size() )
//        printf("One insertable pin connect more than one holes!! May contains collision in the model\n ");

    return true;
}

bool BeamLayoutConnectorGenerator::noCollision_non_pin_components(Connector& connector, set<BeamHole*>& current_holes, set<BeamHole*>& adj_holes)
//返回当前的Connector的非pin的部分是不是与其他现有的部分有collision
{
    vector<Pin> comp_pins = connector.getCurrentComponentPins();
    for(auto& pin : comp_pins)
    {
        if(pin.insertable)
            continue;
        for(auto& h : current_holes)
        {
            if ((pin.pos-h->pos).norm()<0.9)
                return false;
        }
        for(auto& h : adj_holes)
        {
            if ((pin.pos-h->pos).norm()<0.9)
                return false;
        }
    }
    return true;
}

bool BeamLayoutConnectorGenerator::noCollision_non_pin_components(Connector& connector, set<BeamHole*>& current_holes ){
    vector<Pin> comp_pins = connector.getCurrentComponentPins();
    for(auto& pin : comp_pins)
    {
        if(pin.insertable)
            continue;
        for(auto& h : current_holes)
        {
            if ((pin.pos-h->pos).norm()<0.9)
                return false;
        }
    }
    return true;
}

bool BeamLayoutConnectorGenerator::bridgeTwoComponents(Connector &connector, set<BeamHole *> & current_holes,
                                                       set<BeamHole *> & adj_holes){
    bool cover_current_comp = false;
    bool cover_adj_comp     = false;

    for(auto& h : connector.connected_holes){
        if( current_holes.find(h) != current_holes.end() ) cover_current_comp = true;
        if( adj_holes.find(h)     != adj_holes.end()     ) cover_adj_comp     = true;
    }

    return cover_current_comp && cover_adj_comp;

}