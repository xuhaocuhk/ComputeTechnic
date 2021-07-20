//
// Created by 徐豪 on 2018/6/28.
//

#include <util/util.h>
#include "BeamLayout.h"
#include "util/Helper.h"
#include "objectives/Grader.h"
#include "configure/global_variables.h"
#include <fstream>

BeamLayout::BeamLayout(vector<UnitEdge*>& unit_edges, vector<LayerHole*>& all_holes, Component* layerNode){

    this->unit_edges = unit_edges;
    this->holes = all_holes;
    this->belonged_component = layerNode;

    //Initialize edge to beam mapping
    for(auto& ue : this->unit_edges){
        this->beamsOnEdge.insert(std::pair<UnitEdge*, set<BeamInstance*>>(ue, set<BeamInstance*>()));
    }
    for(auto& h : this->holes){
        this->beamsOnHole.insert(std::pair<LayerHole*, set<BeamInstance*>>(h,  set<BeamInstance*>()));
    }
    for(auto& h : this->holes){
        this->outerBeamsOnHole.insert(std::pair<LayerHole*, set<BeamInstance*>>(h,  set<BeamInstance*>()));
    }

    //Initialize affected holes
    for(auto& h : this->holes){
        map<int, LayerHole*> layerToHole;
        for( int i = - MAX_LAYER; i < MAX_LAYER; i++ ){
            layerToHole.insert( std::pair< int, LayerHole* >(i, NULL) );
        }
        this->affected_hole.insert( std::pair<LayerHole*, map<int, LayerHole*> >(h, layerToHole) );
    }

}

bool BeamLayout::allEdgeCovered(){
    for(auto& ue : this->unit_edges){
        if(beamsOnEdge[ue].empty())
            return false;
    }
    return true;
}

bool BeamLayout::allNonCollisionEdgesCovered(){
    for(auto& ue : this->unit_edges){
        if( outerBeamsOnHole[ue->hole1].empty() && outerBeamsOnHole[ue->hole2].empty() && beamsOnEdge[ue].empty() )
            return false;
    }
    return true;
}


vector<UnitEdge*> BeamLayout::unCoveredEdges(){
    vector<UnitEdge*> result;
    for(auto& ue : this->unit_edges){
        if(beamsOnEdge[ue].empty())
            result.push_back(ue);
    }
    return result;
}

vector<UnitEdge*> BeamLayout::unCoveredNonCollisionEdges(){
    vector<UnitEdge*> result;
    for(auto& ue : this->unit_edges){
        if( outerBeamsOnHole[ue->hole1].empty() && outerBeamsOnHole[ue->hole2].empty() && beamsOnEdge[ue].empty() )
            result.push_back(ue);
    }
    return result;
}

void BeamLayout::random_init(){
    //Randomly cover all edges
    while(!allEdgeCovered()){
        vector<UnitEdge*> uc_edges = unCoveredEdges();
        UnitEdge* unitEdge = *uc_edges.begin(); // pick an uncovered edge
        auto chosen_beam = unitEdge->possibleBeams.at(intRandom(0,unitEdge->possibleBeams.size()-1)); // Pick a random beam
        BeamInstance* beamInstance = new BeamInstance(chosen_beam);
        this->addBeam(beamInstance);
    }
    //Greedily assign layer number
    (*this->beams.begin())->layer_number = 0;//先设置一个
    arrangeNoLayerBeamsLayer_Greedy(this->beams);
}

void BeamLayout::random_init_symm(){

    //Randomly cover all edges
    while(!allEdgeCovered()){
        vector<UnitEdge*> uc_edges = unCoveredEdges();
        UnitEdge* unitEdge = *uc_edges.begin(); // pick an uncovered edge
        auto chosen_beam = unitEdge->possibleBeams.at(intRandom(0, unitEdge->possibleBeams.size()-1)); // Pick a random beam
        BeamInstance* beamInstance = new BeamInstance(chosen_beam);
        this->addBeam(beamInstance);
        //add symmetry beams
        if(chosen_beam->refl_symmetry_beam != NULL && chosen_beam != chosen_beam->refl_symmetry_beam){
            BeamInstance* beamInstance_symm = new BeamInstance(chosen_beam->refl_symmetry_beam);
            beamInstance->symm_beam = beamInstance_symm;
            beamInstance_symm->symm_beam = beamInstance;
            this->addBeam(beamInstance_symm);
        }
    }

    //Greedily assign layer number
    randomInitilizeLayers(this->beams);//a more random layer assign function
}


OperationBeams BeamLayout::random_replace_symm(){

    OperationBeams optBeams;

    if(holes.size() == 1){ // handle special case for single-hole-component
        //doing nothing
    }else{
        /* 1. choose an uncovered edge */
        UnitEdge* chosenEdge = this->unit_edges.at(intRandom(0,unit_edges.size()-1));

        /* 2. remove all beams passing through this edge and the end point of this edge  */
        optBeams = eraseBeamsOnEdgeAndHole(optBeams, chosenEdge);

        /* 3. randomly choose beams covering current uncovered unit */
        pickRandomBeams_allCovered(optBeams);

        /* 4. greedily assign layers for beams */
        arrangeNoLayerBeamsLayer_Greedy(this->beams);
//        arrangeAllBeamsLayer_Greedy(this->beams);
    }
    return optBeams;
}

OperationBeams BeamLayout::random_replace_symm_weighted_prob(){

    OperationBeams optBeams;

    if(holes.size() == 1){ // handle special case for single-hole-component
        //doing nothing
    }else{
        /* 1. choose an uncovered edge */
        UnitEdge* chosenEdge = this->unit_edges.at(intRandom(0,unit_edges.size()-1));

        /* 2. remove all beams passing through this edge and the end point of this edge  */
        optBeams = eraseBeamsOnEdgeAndHole(optBeams, chosenEdge);

        /* 3. randomly choose beams covering current uncovered unit */
        pickRandomBeamsWithProbability_allCovered(optBeams);

        /* 4. greedily assign layers for beams */
        arrangeNoLayerBeamsLayer_Greedy(this->beams);
//        arrangeAllBeamsLayer_Greedy(this->beams);
    }
    return optBeams;
}

OperationBeams BeamLayout::random_replace_symm_weighted_prob_non_collision(){
    OperationBeams optBeams;

    if(holes.size() == 1){ // handle special case for single-hole-component
        if( this->outerBeamsOnHole[this->holes.front()].empty() && this->beams.empty() ){
            //put single hole here
            auto single_beam = std::find_if(this->holes.front()->possible_beams.begin(), this->holes.front()->possible_beams.end(),
                                            [](BeamTemplate *b) {
                                                return b->name == "TB1";
                                            });
            assert( single_beam != this->holes.front()->possible_beams.end() ); //must be found

            BeamInstance* beamInstance = new BeamInstance( *single_beam );
            beamInstance->layer_number = 0;
            optBeams.newBeams.push_back(beamInstance);
            this->addBeam(beamInstance);
        }else if( !this->outerBeamsOnHole[this->holes.front()].empty() && !this->beams.empty() ) {
            //erase beam
            optBeams.oldBeams.push_back( *this->beams.begin() );
            eraseBeam( *this->beams.begin() );
        }else{
            //do nothing
        }

    }else{

        /* 2. remove all beams passing through this edge and the end point of this edge  */
        optBeams = eraseBeamsOnEdgeAndHole_affect(optBeams);

        /* 3. randomly choose beams covering current uncovered unit */
        pickRandomBeamsWithProbabiltiy_allCovered_affect(optBeams);

        /* 4. greedily assign layers for beams */
        arrangeNoLayerBeamsLayer_Greedy(this->beams);

        /* 5. update affect information */
        updateOuterBeams(optBeams);

        ////Self symmetry condition
        if( this->belonged_component->layerGraph->sketchGroup->isSelfSymmetry() ) {
            updateSelfSymmetryInfo(optBeams);
        }
    }
    return optBeams;

}

OperationBeams BeamLayout::random_replace_symm_weighted_prob_non_collision_connectness( UnitEdge* ucEdge ){

    OperationBeams optBeams;

    if(holes.size() == 1){ // handle special case for single-hole-component

        if( this->outerBeamsOnHole[this->holes.front()].empty() && this->beams.empty() ){
            //put single hole here
            auto single_beam = std::find_if(this->holes.front()->possible_beams.begin(), this->holes.front()->possible_beams.end(),
                                            [](BeamTemplate *b) {
                                                return b->name == "TB1";
                                            });
            assert( single_beam != this->holes.front()->possible_beams.end() ); //must be found

            BeamInstance* beamInstance = new BeamInstance( *single_beam );
            beamInstance->layer_number = 0;
            optBeams.newBeams.push_back(beamInstance);
            this->addBeam(beamInstance);
        }else if( !this->outerBeamsOnHole[this->holes.front()].empty() && !this->beams.empty() ) {
            //erase beam
            optBeams.oldBeams.push_back( *this->beams.begin() );
            eraseBeam( *this->beams.begin() );
        }else{
            //do nothing
        }

    }else{

        /* 2. remove all beams passing through this edge and the end point of this edge  */
        optBeams = eraseBeamsOnEdgeAndHole(optBeams, ucEdge);

        /* 3. randomly choose beams covering current uncovered unit */
        pickRandomBeamsWithProbabiltiy_allCovered_affect(optBeams);

        /* 4. greedily assign layers for beams */
        arrangeNoLayerBeamsLayer_Greedy(this->beams);

        /* 5. update affect information */
        updateOuterBeams(optBeams);

        ////Self symmetry condition
        if( this->belonged_component->layerGraph->sketchGroup->isSelfSymmetry() ) {
            updateSelfSymmetryInfo(optBeams);
        }
    }
    return optBeams;
}

OperationBeams BeamLayout::random_replace_symm_connectivity( vector<UnitEdge*>& consider_edges ) // operation to adjust connectivity
{
    OperationBeams optBeams;

    double operation_seed = cRandom(0,1);

    if(operation_seed < 0.7){
        printf("operation: remove and add beam\n");
        eraseRandomSingleBeam(optBeams, consider_edges);
        pickRandomSingleBeam(optBeams, consider_edges);
    }else if (operation_seed >= 0.7 && operation_seed <= 0.85 ){
        printf("operation: add  beam\n");
        pickRandomSingleBeam(optBeams, consider_edges);
    }else {
        printf("operation: erase beam\n");
        eraseRandomSingleBeam(optBeams, consider_edges);
    }

    /* 3. greedily assign layers for beams */
//    arrangeNoLayerBeamsLayer_Greedy(this->beams); //TODO: when the newly added beam not adjacent to another other, problem appears
    arrangeNoLayerBeamsLayer_Greedy(optBeams);

    return optBeams;
}

void BeamLayout::pickRandomSingleBeam(OperationBeams & optBeams, vector<UnitEdge*>& consider_edges){

    if(this->holes.size() == 1){

        auto single_beam = std::find_if(this->holes.front()->possible_beams.begin(), this->holes.front()->possible_beams.end(),
                                        [](BeamTemplate *b) {
            return b->name == "TB1";
        });
        assert( single_beam != this->holes.front()->possible_beams.end() ); //must be found

        BeamInstance* beamInstance = new BeamInstance( *single_beam );
        beamInstance->layer_number = 0;
        optBeams.newBeams.push_back(beamInstance);
        this->addBeam(beamInstance);

    }else{
        if(allEdgeCovered()){
            //do nothing ...
        }else{
            vector<UnitEdge*> uc_edges = unCoveredEdges();
            UnitEdge* unitEdge = *uc_edges.begin(); // pick an uncovered edge
            auto chosen_beam = unitEdge->possibleBeams.at(intRandom(0, unitEdge->possibleBeams.size()-1)); // Pick a random beam
            BeamInstance* beamInstance = new BeamInstance(chosen_beam);
            optBeams.newBeams.push_back(beamInstance);
            addBeam(beamInstance);
            //add symmetry beams
            if(chosen_beam->refl_symmetry_beam != NULL && chosen_beam->refl_symmetry_beam != chosen_beam){
                BeamInstance* beamInstance_symm = new BeamInstance(chosen_beam->refl_symmetry_beam);
                beamInstance->symm_beam = beamInstance_symm;
                beamInstance_symm->symm_beam = beamInstance;
                optBeams.newBeams.push_back(beamInstance_symm);
                addBeam(beamInstance_symm);
            }
        }
    }

}

void BeamLayout::pickRandomBeams_allCovered(OperationBeams &optBeams) {

    while(!allEdgeCovered()){
        vector<UnitEdge*> uc_edges = unCoveredEdges();
        UnitEdge* unitEdge = *uc_edges.begin(); // pick an uncovered edge
        auto chosen_beam = unitEdge->possibleBeams.at(intRandom(0,unitEdge->possibleBeams.size()-1)); // Pick a random beam
        BeamInstance* beamInstance = new BeamInstance(chosen_beam);
        optBeams.newBeams.push_back(beamInstance);
        addBeam(beamInstance);
        //add symmetry beams
        if(chosen_beam->refl_symmetry_beam != NULL && chosen_beam->refl_symmetry_beam != chosen_beam){
            BeamInstance* beamInstance_symm = new BeamInstance(chosen_beam->refl_symmetry_beam);
            beamInstance->symm_beam = beamInstance_symm;
            beamInstance_symm->symm_beam = beamInstance;
            optBeams.newBeams.push_back(beamInstance_symm);
            addBeam(beamInstance_symm);
        }
    }

}

void BeamLayout::pickRandomBeamsWithProbability_allCovered(OperationBeams &optBeams) {

    while(!allEdgeCovered()){ //TODO: accelarate-> dont need to check all unit edges here
        vector<UnitEdge*> uc_edges = unCoveredEdges();
        UnitEdge* unitEdge = *uc_edges.begin(); // pick an uncovered edge

        /**** Choose beam according to uncovered beams ****/
        vector<int> weight;
        for(int i = 0 ; i<unitEdge->possibleBeams.size(); i++){
            int uncoverd_edge_num = 0;

            for(auto& u : unitEdge->possibleBeams[i]->covered_edges)
                if(beamsOnEdge[u].empty())
                    uncoverd_edge_num ++;

            weight.push_back(uncoverd_edge_num);

        }

        auto chosen_beam = unitEdge->possibleBeams[ pickWeightedRandElement(weight) ]; // Pick a according to a weight
        BeamInstance* beamInstance = new BeamInstance(chosen_beam);
        optBeams.newBeams.push_back(beamInstance);
        addBeam(beamInstance);
        //add symmetry beams
        if(chosen_beam->refl_symmetry_beam != NULL && chosen_beam->refl_symmetry_beam != chosen_beam){
            BeamInstance* beamInstance_symm = new BeamInstance(chosen_beam->refl_symmetry_beam);
            beamInstance->symm_beam = beamInstance_symm;
            beamInstance_symm->symm_beam = beamInstance;
            optBeams.newBeams.push_back(beamInstance_symm);
            addBeam(beamInstance_symm);
        }
    }
}

void BeamLayout::pickRandomBeamsWithProbabiltiy_allCovered_affect(OperationBeams & optBeams) //pick beams without collision with outside components
{

    while(!allNonCollisionEdgesCovered()){ //TODO: accelarate-> dont need to check all unit edges here
        vector<UnitEdge*> uc_edges = unCoveredNonCollisionEdges();

        UnitEdge* unitEdge = *uc_edges.begin(); // pick an uncovered edge

        /**** Choose beam according to uncovered beams ****/
        vector<BeamTemplate *> possibleBeams = unitEdge->possibleBeams;

        possibleBeams.erase(std::remove_if(possibleBeams.begin(), possibleBeams.end(), [this](BeamTemplate *b) {
            for(auto& h : b->covered_holes){
                if( !this->outerBeamsOnHole[h].empty() )
                    return true;
            }
            return false;
        }), possibleBeams.end());


        vector<int> weight;
        for(int i = 0 ; i < possibleBeams.size(); i++){
            int uncoverd_edge_num = 0;

            for(auto& u : possibleBeams[i]->covered_edges)
                if(beamsOnEdge[u].empty())
                    uncoverd_edge_num ++;

            weight.push_back(uncoverd_edge_num);

        }

        auto chosen_beam = possibleBeams[ pickWeightedRandElement(weight) ]; // Pick a according to a weight
        BeamInstance* beamInstance = new BeamInstance(chosen_beam);
        optBeams.newBeams.push_back(beamInstance);
        addBeam(beamInstance);
        //add symmetry beams
        if(chosen_beam->refl_symmetry_beam != NULL && chosen_beam->refl_symmetry_beam != chosen_beam){
            BeamInstance* beamInstance_symm = new BeamInstance(chosen_beam->refl_symmetry_beam);
            beamInstance->symm_beam = beamInstance_symm;
            beamInstance_symm->symm_beam = beamInstance;
            optBeams.newBeams.push_back(beamInstance_symm);
            addBeam(beamInstance_symm);
        }
    }

}

bool dominate(BeamTemplate *new_beam, BeamTemplate *old_beam) {//check whether new beam dominate the old beam
    for(auto& oe : old_beam->covered_edges){
        if(std::find(new_beam->covered_edges.begin(), new_beam->covered_edges.end(), oe) == new_beam->covered_edges.end())//not in it
            return false;
    }
    return true;
}

void BeamLayout::pickRandomBeamsNoDominate_allCovered(OperationBeams &optBeams){
    while(!allEdgeCovered()){
        vector<UnitEdge*> uc_edges = unCoveredEdges();
        UnitEdge* unitEdge = uc_edges.front(); // pick an uncovered edge
        vector<BeamTemplate *> nonDominateBeams = unitEdge->possibleBeams;
        int repeat_num = 0;
        regenerate:
        auto chosen_beam = unitEdge->possibleBeams.at(intRandom(0,unitEdge->possibleBeams.size()-1)); // Pick a random beam
        for(auto& e : chosen_beam->covered_edges){
            for(auto& b : beamsOnEdge[e]){
                if(dominate(chosen_beam, b->template_beam)){
                    repeat_num++;
                    goto regenerate;
                }
            }
        }
        printf("repeat:%d\n",repeat_num);
        BeamInstance* beamInstance = new BeamInstance(chosen_beam);
        optBeams.newBeams.push_back(beamInstance);
        addBeam(beamInstance);
        //add symmetry beams
        if(chosen_beam->refl_symmetry_beam != NULL && chosen_beam->refl_symmetry_beam != chosen_beam){
            BeamInstance* beamInstance_symm = new BeamInstance(chosen_beam->refl_symmetry_beam);
            beamInstance->symm_beam = beamInstance_symm;
            beamInstance_symm->symm_beam = beamInstance;
            optBeams.newBeams.push_back(beamInstance_symm);
            addBeam(beamInstance_symm);
        }
    }
}

OperationBeams& BeamLayout::eraseRandomSingleBeam(OperationBeams &optBeams, vector<UnitEdge* > & consider_edges){

    if(this->holes.size() == 1){
        if( !this->beams.empty() ){
            optBeams.oldBeams.push_back( *this->beams.begin() );
            eraseBeam( *this->beams.begin() );
        }
    }else{
        set<BeamInstance*> m_beams;
        for(auto& ue : consider_edges){
            m_beams.insert( beamsOnEdge[ue].begin(), beamsOnEdge[ue].end() );
        }

        if(m_beams.empty()){
            //do nothing
        }else{
            auto iter = m_beams.begin();
            std::advance(iter, intRandom(0, m_beams.size()-1 ));
            optBeams.oldBeams.push_back(*iter);
            eraseBeam(*iter);
            if( (*iter)->symm_beam != NULL ){
                optBeams.oldBeams.push_back((*iter)->symm_beam);
                eraseBeam((*iter)->symm_beam);
            }
        }
    }

    return optBeams;

}

OperationBeams& BeamLayout::eraseBeamsOnEdgeAndHole(OperationBeams &optBeams, UnitEdge *chosenEdge) {

    /**** all beams in unit edge and it's two end points ****/
    set<BeamInstance*> delete_beams = this->beamsOnEdge[chosenEdge];
    delete_beams.insert(beamsOnHole[chosenEdge->hole1].begin(), beamsOnHole[chosenEdge->hole1].end());
    delete_beams.insert(beamsOnHole[chosenEdge->hole2].begin(), beamsOnHole[chosenEdge->hole2].end());

    //TODO : optimize the following code using beams's symmetry information
    //add symmetry to be deleted beams
    if(chosenEdge->symm_uedge != NULL){
        delete_beams.insert(beamsOnEdge[chosenEdge->symm_uedge].begin(), beamsOnEdge[chosenEdge->symm_uedge].end());
        delete_beams.insert(beamsOnHole[chosenEdge->symm_uedge->hole1].begin(), beamsOnHole[chosenEdge->symm_uedge->hole1].end());
        delete_beams.insert(beamsOnHole[chosenEdge->symm_uedge->hole2].begin(), beamsOnHole[chosenEdge->symm_uedge->hole2].end());
    }

    optBeams.oldBeams = vector<BeamInstance*>(delete_beams.begin(),delete_beams.end());
    for(auto& b : delete_beams) eraseBeam(b);
    return optBeams;
}

OperationBeams& BeamLayout::eraseBeamsOnEdgeAndHole_affect(OperationBeams& optBeams){

    /*** get random unit edge ***/
    UnitEdge* chosenEdge = this->unit_edges[intRandom(0, this->unit_edges.size()-1)];

    /**** all beams in unit edge and it's two end points ****/
    set<BeamInstance*> delete_beams = this->beamsOnEdge[chosenEdge];
    delete_beams.insert(beamsOnHole[chosenEdge->hole1].begin(), beamsOnHole[chosenEdge->hole1].end());
    delete_beams.insert(beamsOnHole[chosenEdge->hole2].begin(), beamsOnHole[chosenEdge->hole2].end());

    //TODO : optimize the following code using beams's symmetry information
    //add symmetry to be deleted beams
    if(chosenEdge->symm_uedge != NULL){
        delete_beams.insert(beamsOnEdge[chosenEdge->symm_uedge].begin(), beamsOnEdge[chosenEdge->symm_uedge].end());
        delete_beams.insert(beamsOnHole[chosenEdge->symm_uedge->hole1].begin(), beamsOnHole[chosenEdge->symm_uedge->hole1].end());
        delete_beams.insert(beamsOnHole[chosenEdge->symm_uedge->hole2].begin(), beamsOnHole[chosenEdge->symm_uedge->hole2].end());
    }

    optBeams.oldBeams = vector<BeamInstance*>(delete_beams.begin(),delete_beams.end());
    for(auto& b : delete_beams) eraseBeam(b);
    return optBeams;

}

void BeamLayout::updateOuterBeams(OperationBeams& optBeams){
    ////Removing beams
    using std::cout;
    for(auto& del_b : optBeams.oldBeams){
        for(auto& h : del_b->template_beam->covered_holes){
            if( affected_hole[h][del_b->layer_number] != NULL ){
                auto& outer_hole = affected_hole[h][del_b->layer_number];
                outer_hole->belonged_beamLayout->outerBeamsOnHole[outer_hole].erase(del_b);
                if(outer_hole->symmtry_hole != NULL){
                    outer_hole->symmtry_hole->belonged_beamLayout->outerBeamsOnHole[outer_hole->symmtry_hole].erase(del_b);
                }
            }
        }
    }

    ////Adding beams
    for(auto& new_b : optBeams.newBeams){
        for(auto& h : new_b->template_beam->covered_holes){
            if( affected_hole[h][new_b->layer_number] != NULL ){
                auto& outer_hole = affected_hole[h][new_b->layer_number];
                outer_hole->belonged_beamLayout->outerBeamsOnHole[outer_hole].insert(new_b);
                if(outer_hole->symmtry_hole != NULL){
                    outer_hole->belonged_beamLayout->outerBeamsOnHole[outer_hole->symmtry_hole].insert(new_b);
                }
            }
        }
    }
}

void BeamLayout::updateSelfSymmetryInfo(OperationBeams& oprBeams){
    for(auto& del_b : oprBeams.oldBeams){
        for(auto& h : del_b->template_beam->covered_holes){
            if( this->affected_hole[h][-del_b->layer_number] != NULL ){
                auto& outer_hole = this->affected_hole[h][-del_b->layer_number];
                outer_hole->belonged_beamLayout->outerBeamsOnHole[outer_hole].erase(del_b);
                if(outer_hole->symmtry_hole != NULL){
                    outer_hole->symmtry_hole->belonged_beamLayout->outerBeamsOnHole[outer_hole->symmtry_hole].erase(del_b);
                }
            }
        }
    }

    ////Adding beams
    for(auto& new_b : oprBeams.newBeams){
        for(auto& h : new_b->template_beam->covered_holes){
            if( this->affected_hole[h][-new_b->layer_number] != NULL ){
                auto& outer_hole = this->affected_hole[h][-new_b->layer_number];
                outer_hole->belonged_beamLayout->outerBeamsOnHole[outer_hole].insert(new_b);
                if(outer_hole->symmtry_hole != NULL){
                    outer_hole->belonged_beamLayout->outerBeamsOnHole[outer_hole->symmtry_hole].insert(new_b);
                }
            }
        }
    }
}

void BeamLayout::undoOperation(OperationBeams& operation){
    for(auto& b : operation.newBeams){
        eraseBeam(b);
    }
    for(auto& b : operation.oldBeams){
        addBeam(b);
    }
}

void BeamLayout::addBeam(BeamInstance* beamInstance){
    this->beams.insert(beamInstance);
    for(auto& ue : beamInstance->template_beam->covered_edges){
        beamsOnEdge[ue].insert(beamInstance);
    }
    for(auto& h : beamInstance->template_beam->covered_holes){
        beamsOnHole[h].insert(beamInstance);
    }
}

void BeamLayout::eraseBeam(BeamInstance* beam){

    this->beams.erase(beam);

    for(auto& u_edge : beam->template_beam->covered_edges){
        beamsOnEdge[u_edge].erase(beam);
    }
    for(auto& hole : beam->template_beam->covered_holes){
        beamsOnHole[hole].erase(beam);
    }
}

std::pair<int, int> BeamLayout::chooseBestLayer_insideBound(BeamInstance &beamInstance,
                                                            std::pair<int, int> &layer_bound){
    assert(beamInstance.layer_number == NO_LAYER);
    int former_layer = beamInstance.layer_number;
    set<int> exists_layers;
    for (const auto& hole : beamInstance.template_beam->covered_holes) {
        for(auto& b : beamsOnHole[hole]){
            if(b != &beamInstance && b->layer_number != NO_LAYER)
                exists_layers.insert(b->layer_number);
        }
        for(auto& clo_hole : hole->close_holes){
            for(auto& b : beamsOnHole[clo_hole]){
                if(b != &beamInstance && b->layer_number != NO_LAYER)
                    exists_layers.insert(b->layer_number);
            }
        }
    }

    if(exists_layers.size() == 0){
        beamInstance.layer_number = 0;
    }else{
        auto max = std::max_element(exists_layers.begin(),exists_layers.end());
        auto min = std::min_element(exists_layers.begin(),exists_layers.end());
        if((*max-*min) == exists_layers.size()-1)//说明没有断层
        {
            vector<int> possi_inside_layers;
            if((*max+1)<=layer_bound.second) possi_inside_layers.push_back(*max+1);
            if((*min-1)>=layer_bound.first) possi_inside_layers.push_back(*min-1);
            if(possi_inside_layers.size() == 0 || possi_inside_layers.size() == 2){
                beamInstance.layer_number = abs(*max + 1) < abs(*min - 1) ? *max + 1 : *min - 1;
            }else{ //possi_inside_layers.size()==1
                beamInstance.layer_number = possi_inside_layers.back();
            }
            if(beamInstance.layer_number > layer_bound.second) layer_bound.second = beamInstance.layer_number;
            if(beamInstance.layer_number < layer_bound.first)  layer_bound.first = beamInstance.layer_number;
        }else{//如果有断层则填上断层的数
            for(int i = *min;i<=*max ;i++)
            {
                auto search = exists_layers.find(i);
                if(search == exists_layers.end()) {
                    beamInstance.layer_number = beamInstance.layer_number == NO_LAYER? i :
                                                ( abs(i) < abs(beamInstance.layer_number) ? i : beamInstance.layer_number );
                }
            }
        }
    }
    assert(beamInstance.layer_number != NO_LAYER);
    return layer_bound;
};

void BeamLayout::chooseBestLayer(BeamInstance& beamInstance){
    assert(beamInstance.layer_number == NO_LAYER);
    int former_layer = beamInstance.layer_number;
    set<int> exists_layers;
    for (const auto& hole : beamInstance.template_beam->covered_holes) {
        for(auto& b : beamsOnHole[hole]){
            if(b != &beamInstance && b->layer_number != NO_LAYER)
                exists_layers.insert(b->layer_number);
        }
        for(auto& clo_hole : hole->close_holes){
            for(auto& b : beamsOnHole[clo_hole]){
                if(b != &beamInstance && b->layer_number != NO_LAYER)
                    exists_layers.insert(b->layer_number);
            }
        }
    }

    if(exists_layers.size() == 0){
        beamInstance.layer_number = 0;
    }else{
        auto max = std::max_element(exists_layers.begin(),exists_layers.end());
        auto min = std::min_element(exists_layers.begin(),exists_layers.end());
        if((*max-*min) == exists_layers.size()-1)//说明没有断层
        {
            //greedily assign layer to 0 layer number
            beamInstance.layer_number = abs(*max + 1) < abs(*min - 1) ? *max + 1 : *min - 1;
        }else{//如果有断层则填上断层的数
            for(int i = *min;i<=*max ;i++)
            {
                auto search = exists_layers.find(i);
                if(search == exists_layers.end()) {
                    beamInstance.layer_number = i;
                }
            }
        }
    }
}

void BeamLayout::chooseRandomLayer(BeamInstance& beamInstance){
    assert(beamInstance.layer_number == NO_LAYER);
    set<int> exists_layers;
    for (const auto& hole : beamInstance.template_beam->covered_holes) {
        for(auto& b : beamsOnHole[hole]){
            if(b != &beamInstance && b->layer_number != NO_LAYER)
                exists_layers.insert(b->layer_number);
        }
        for(auto& clo_hole : hole->close_holes){
            for(auto& b : beamsOnHole[clo_hole]){
                if(b != &beamInstance && b->layer_number != NO_LAYER)
                    exists_layers.insert(b->layer_number);
            }
        }
    }

    if(exists_layers.size() == 0){
        beamInstance.layer_number = 0;
    }else{
        auto max = std::max_element(exists_layers.begin(),exists_layers.end());
        auto min = std::min_element(exists_layers.begin(),exists_layers.end());
        vector<int> available_layers;
        for(int l = *min-1; l<=*max+1; l++){
            if(exists_layers.find(l) == exists_layers.end()){
                available_layers.push_back(l);
            }
        }
        beamInstance.layer_number = available_layers[intRandom(0,available_layers.size()-1)];
    }
}

void BeamLayout::randomInitilizeLayers(set<BeamInstance*, BeamInstance::BeamInstanceComp>& m_beams){
    (*m_beams.begin())->layer_number = 0;
    bool changed_beam = true;
    while(changed_beam == true){
        changed_beam = false;
        for(auto& b : m_beams){
            if(b->layer_number == NO_LAYER && isAdjToLayerBeam(*b)){
                chooseRandomLayer(*b);
                changed_beam = true;
            }
        }
    }
}

void BeamLayout::randomInitilizeLayers(vector<BeamInstance*>& m_beams){
    m_beams.back()->layer_number = 0;
    bool changed_beam = true;
    while(changed_beam == true){
        changed_beam = false;
        for(auto& b : m_beams){
            if(b->layer_number == NO_LAYER && isAdjToLayerBeam(*b)){
                chooseRandomLayer(*b);
                changed_beam = true;
            }
        }
    }
}

void BeamLayout::arrangeNoLayerBeamsLayer_Greedy(set<BeamInstance *, BeamInstance::BeamInstanceComp> &m_beams){
    if (m_beams.empty())
        return;

    int lower_bound = 99999, upper_bound = -99999;
    for(auto& b : m_beams) {
        if(b->layer_number!=NO_LAYER){
            if(b->layer_number>upper_bound) upper_bound = b->layer_number;
            if(b->layer_number<lower_bound) lower_bound = b->layer_number;
        }
    }
    std::pair<int,int> layer_bound( lower_bound , upper_bound );
    bool changed_beam = true;
    while(changed_beam == true){
        changed_beam = false;
        for(auto& b : m_beams){
            if(b->layer_number == NO_LAYER && isAdjToLayerBeam(*b)){
                layer_bound = chooseBestLayer_insideBound(*b, layer_bound);
                changed_beam = true;
            }
        }
    }


    if((*m_beams.begin())->layer_number == NO_LAYER)//still have beams that is no layer, 此时说明*所有*beam没有assign layer
    {
        (*m_beams.begin())->layer_number = 0;
        arrangeNoLayerBeamsLayer_Greedy(m_beams);
    }else{ // When containing beams without layer, and
        auto find_nolayer = std::find_if(m_beams.begin(), m_beams.end(),
                                         [](BeamInstance* beamInstance){ return beamInstance->layer_number == NO_LAYER; } );
        if( find_nolayer != m_beams.end() ){
            (*find_nolayer)->layer_number = 0;
            arrangeNoLayerBeamsLayer_Greedy(m_beams);
        }
    }

}

void BeamLayout::arrangeNoLayerBeamsLayer_Greedy (OperationBeams& oprBeams) //only do layer arranging for the beams just added
{
    int lower_bound = 99999, upper_bound = -99999;
    for(auto& b : this->beams) {
        if(b->layer_number!=NO_LAYER){
            if(b->layer_number>upper_bound) upper_bound = b->layer_number;
            if(b->layer_number<lower_bound) lower_bound = b->layer_number;
        }
    }
    assert(lower_bound > -100 && upper_bound < 100);
    std::pair<int,int> layer_bound(lower_bound,upper_bound);

    for(auto& b : oprBeams.newBeams){
        if( isAdjToLayerBeam(*b) ){
            chooseBestLayer_insideBound( *b, layer_bound );
        }else{
            b->layer_number = 0;
        }
    }
}

void BeamLayout::arrangeNoLayerBeamsLayer_Greedy(vector<BeamInstance *> &m_beams){
    int lower_bound = 99999, upper_bound = -99999;
    for(auto& b : m_beams) {
        if(b->layer_number!=NO_LAYER){
            if(b->layer_number>upper_bound) upper_bound = b->layer_number;
            if(b->layer_number<lower_bound) lower_bound = b->layer_number;
        }
    }
    assert(lower_bound > -100 && upper_bound < 100);
    std::pair<int,int> layer_bound(lower_bound,upper_bound);
    bool changed_beam = true;
    while(changed_beam == true){
        changed_beam = false;
        for(auto& b : m_beams){
            if(b->layer_number == NO_LAYER && isAdjToLayerBeam(*b)){
                layer_bound = chooseBestLayer_insideBound(*b, layer_bound);
                changed_beam = true;
            }
        }
    }
    if(m_beams.back()->layer_number == NO_LAYER)//still have beams that is no layer, 此时说明*所有*beam没有assign layer
    {
        m_beams.back()->layer_number = 0;
        arrangeNoLayerBeamsLayer_Greedy(m_beams);
    }
}


bool BeamLayout::isAdjToLayerBeam(BeamInstance &m_beam)//返回这个beam是不是没有layer并且与一个有layer的beam挨着
{
    assert(m_beam.layer_number == NO_LAYER);
    for(auto& h : m_beam.template_beam->covered_holes){
        for(auto& adj_b : beamsOnHole[h]){
             if (adj_b->layer_number != NO_LAYER)
                 return true;
        }
    }
    return false;
}

bool BeamLayout::isFloatingOnAir(BeamInstance & m_beam){
    assert(m_beam.layer_number != NO_LAYER);
    set<int> adj_beam_layers; //layer number of all adjacent beams

    for(auto& h : m_beam.template_beam->covered_holes){
        for(auto& adj_b : beamsOnHole[h]){
            if (adj_b != &m_beam && adj_b->layer_number != NO_LAYER)
                adj_beam_layers.insert(adj_b->layer_number);
        }
    }

    if(adj_beam_layers.empty()){
        return false;
    }else{
        return adj_beam_layers.find(m_beam.layer_number + 1) == adj_beam_layers.end() &&
               adj_beam_layers.find(m_beam.layer_number - 1) == adj_beam_layers.end() ;
    }

}

//DFS search unconnected beam holes, return if the size is enough
void DFS_rigid_edges(UnitEdge *ue, set<UnitEdge*> &result, BeamLayout& beamLayout)
{
    result.insert(ue);
    set<UnitEdge*> adj_edges = ue->getAdjacentEdges();
    for(auto& e : adj_edges){
        //1.需要在这个component上 2)rigid 3)当前数据集中还没有
        if(beamLayout.beamsOnEdge.count(e)==1 &&
           beamLayout.isRigidConnected(ue,e) &&
           result.find(e) == result.end()
        )
           DFS_rigid_edges(e,result, beamLayout);
    }
}

//DFS search unconnected beam holes, return if the size is enough
void DFS_rigid_beams(BeamInstance *bi, set<BeamInstance*> &result, BeamLayout& beamLayout)
{
    result.insert(bi);
    set<BeamInstance*> adj_beams = beamLayout.getAdjRigidConnectBeams(*bi);

    for(auto& e : adj_beams){
        //需要当前数据集中还没有
        if(result.find(e) == result.end())
            DFS_rigid_beams(e,result, beamLayout);
    }
}

set<UnitEdge*> BeamLayout::getAllRigidConnectedEdges(UnitEdge* unitEdge)
{
    set<UnitEdge*> result;
    DFS_rigid_edges(unitEdge,result,*this);
    return result;
}

set<BeamInstance*>  BeamLayout::getAllRigidConnectedBeams(BeamInstance* beamInstance)
{
    set<BeamInstance*> result;
    DFS_rigid_beams(beamInstance, result, *this);
    return result;
}

//not including self
set<BeamInstance*> BeamLayout::getAdjRigidConnectBeams(BeamInstance &beamInstance){

    set<BeamInstance*> adj_beams;

    for(auto& ue : beamInstance.template_beam->covered_edges){
        for(auto& adj_b : beamsOnEdge[ue]){
            if(adj_b != &beamInstance)
                adj_beams.insert(adj_b);
        }
    }

    return adj_beams;
}

bool BeamLayout::isRigidConnected(UnitEdge* u1, UnitEdge* u2)
{
    assert(u1->isAdjacent(u2));
    assert(beamsOnEdge.count(u1) == 1);
    assert(beamsOnEdge.count(u2) == 1);
    auto beams1 = beamsOnEdge[u1];
    auto beams2 = beamsOnEdge[u2];
    vector<BeamInstance*> intersect;
    set_intersection(beams1.begin(),beams1.end(),
                     beams2.begin(),beams2.end(),
                     std::back_inserter(intersect));
    if(!intersect.empty())
        return true;
    return false;
}

bool BeamLayout::connectedOnHole(LayerHole* hole){
    vector<int> numbers;
    for(auto& beam : beamsOnHole[hole])
    {
        if(beam->layer_number == NO_LAYER) continue;
        numbers.push_back(beam->layer_number);
    }
    if(numbers.size()==0)
        return true;
    //判断无重复
    int origin_size = numbers.size();
    std::sort(numbers.begin(),numbers.end());
    auto last = std::unique(numbers.begin(), numbers.end());
    numbers.erase(last, numbers.end());
    if(numbers.size()!=origin_size)
        return false;
    else{//判断无断层
        auto max = std::max_element(numbers.begin(),numbers.end());
        auto min = std::min_element(numbers.begin(),numbers.end());
        if((*max - *min)!=numbers.size()-1)
            return false;
    }
    return true;
}

int BeamLayout::gapNumber(LayerHole* hole){
    vector<int> numbers;
    for(auto& beam : beamsOnHole[hole])
    {
        if(beam->layer_number == NO_LAYER) continue;
        numbers.push_back(beam->layer_number);
    }
    if(numbers.size()==0)
        return 0;
    auto max = std::max_element(numbers.begin(),numbers.end());
    auto min = std::min_element(numbers.begin(),numbers.end());
    return (*max - *min) - numbers.size() +1;
}

//TODO::not consider NO_LAYER yet
int BeamLayout::gapNumber(vector<int>& numbers){
    if(numbers.size()==0)
        return 0;
    auto max = std::max_element(numbers.begin(),numbers.end());
    auto min = std::min_element(numbers.begin(),numbers.end());
    return (*max - *min) - numbers.size() +1;
}

void BeamLayout::fillGap() //fill gaps for the current beams
{
    for(auto& h : holes){
        if( gapNumber(h) > 0 ){

            vector<int> numbers;
            for(auto& beam : beamsOnHole[h])
            {
                if(beam->layer_number == NO_LAYER) continue;
                numbers.push_back(beam->layer_number);
            }

            if(numbers.size()==0)
                continue;
            else{
                auto max = std::max_element(numbers.begin(),numbers.end());
                auto min = std::min_element(numbers.begin(),numbers.end());
                ////Filling vacant holes
                auto single_beam = std::find_if(h->possible_beams.begin(), h->possible_beams.end(),
                                                [](BeamTemplate *b) {
                    return b->name == "TB1";
                });
                assert(single_beam != h->possible_beams.end());

                for(int i = *min;i<=*max ;i++)
                {
                    auto search = std::find(numbers.begin(), numbers.end(), i);
                    if(search == numbers.end()) {
                        BeamInstance* beamInstance = new BeamInstance( *single_beam );
                        beamInstance->layer_number = i;
                        this->addBeam(beamInstance);
                    }
                }

            }

        }
    }



}

BeamLayout* BeamLayout::copySelf()//deep copy a version of self
{
    BeamLayout* newLayout = new BeamLayout(this->unit_edges,this->holes, this->belonged_component);
    std::map<BeamInstance*, BeamInstance*> oldToNewBeams;
    for(auto& b : this->beams){//copy new beams and store they copy
        BeamInstance* new_beam = new BeamInstance(*b);
        oldToNewBeams.insert(std::make_pair(b,new_beam));
        newLayout->beams.insert(new_beam);
    }

    for(auto& bOnh : this->beamsOnHole){
        for(auto& bi : bOnh.second){
            newLayout->beamsOnHole[bOnh.first].insert(oldToNewBeams[bi]);
        }
    }

    for(auto& bOne : this->beamsOnEdge){
        for(auto& bi : bOne.second){
            newLayout->beamsOnEdge[bOne.first].insert(oldToNewBeams[bi]);
        }
    }

    //copy symmetry info
    for(auto& b : this->beams){
        if(b->symm_beam != NULL){
            oldToNewBeams[b]->symm_beam = oldToNewBeams[b->symm_beam];
        }
    }

    return newLayout;
}



