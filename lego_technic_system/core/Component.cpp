//
// Created by 徐豪 on 2018/5/19.
//

#include <iostream>
#include "../bricks/BrickFactory.h"
#include "Component.h"
#include "configure/global_variables.h"
#include "util/util.h"
#include "IO/DataWriter.h"
#include "BeamLayout.h"
#include "util/Timer.h"
#include <fstream>


void Component::gen2DComponentCandidates_TSA_adaptiveK(){
//    assert(this->layerGraph->component_type == LayerGraph::MAJOR_COMPONENT);
//    LayerGraph* layer = this->layerGraph;
//    layer->establishAdjacentRelation(*glob_vars::sketchGraph->getLines());
//    int plane_flag = layer->getCurrentPlaneFlag();
//    layer->pre_compute_all_placement(plane_flag, vector<Beam*>());
//    bool symmetry = layer->establishSymmetryRelation();
//    layer->establishSymmetryUnitEdgeRelation();
//    BeamLayout beamLayout(layer->unit_edges, layer->holes, this);
//    beamLayout.random_init_symm();//should be random enough to make the score relative high
//    DataWriter::writeBeamInstances(beamLayout.beams, "_init_id" + std::to_string(this->_id));
//    string layout_quality;
//    double curr_score = Grader::overall_score_2D(beamLayout, false, layout_quality);
//    double init_score = curr_score;
//    printf("initializetion score:%lf\n", curr_score);
//    cin.get();
//
//    const double TimeLimit = 6;
//    const double ka_min = 580;//the larger, the better the quality is, but much slower
//    const double ka_max = 580;
//
//    const double T_0 = 2000;
//    const double T_min = 0.01;
//    double T = T_0;
//    double M_plus = 0.0;
//    double M_accept = 0;
//    double deltaC_bar = 0.0;
//    int anim_frame = 0;
//    clock_t start = clock();
//    std::ostringstream score_statisctis;
//    std::ostringstream temperature_statisctis;
//
//    double min_score = 99999999.0;
//    BeamLayout* min_layout = beamLayout.copySelf();
//
//    int iteration = 0;
//
//    while(T > T_min){
//        double time_remaining_ratio = (TimeLimit - (double)(clock()-start)/CLOCKS_PER_SEC)/TimeLimit;
//        ++iteration;
//        printf("T:\t %lf M_plus:%lf \t M_accept:%lf \t time_remaining:%lf \n", T, M_plus, M_accept, time_remaining_ratio);
//        printf("current:%lf min:%lf \t iteration:%d\n", curr_score, min_score ,iteration);
//        OperationBeams oprBeams = beamLayout.random_replace_symm();
//        double new_score = Grader::overall_score_2D(beamLayout, false, layout_quality);
//        if(new_score < min_score){
//            min_score = new_score;
//            min_layout = beamLayout.copySelf();
//        }
//        double dt = new_score - curr_score;
//        bool increment = dt > 0;
//        if(exp((-dt)/T) > cRandom(0,1)){
//            //accept this operation
//            deleteElements(oprBeams.oldBeams);
//            curr_score = new_score;
//        }else{
//            beamLayout.undoOperation(oprBeams);
//            deleteElements(oprBeams.newBeams);
//        }
//
//        //update M_plus, M_accept and temperature
//        if(increment)   M_plus   += (dt/T);
//        M_accept = curr_score - init_score;
//
//        deltaC_bar = (deltaC_bar*(iteration-1) + fabs(dt))/iteration;
//        double ka = (1-time_remaining_ratio)*ka_min + time_remaining_ratio*ka_max;
//        printf("ka:%lf\n",ka);
//        T = ( fabs(M_plus) < 1e-10 || M_accept >= 0.0) ? -deltaC_bar/log(0.9995) : (-ka * M_accept)/M_plus;
//
//        /* write statistics */
//        score_statisctis << curr_score << endl;
//        temperature_statisctis<< T << endl;
//
//        /* write annimation frame */
//        if(glob_vars::print_flag && iteration % 300 == 0)
//            DataWriter::writeBeamInstances(beamLayout.beams, "comp_" + std::to_string(this->_id) + "_anim_" + std::to_string(anim_frame++));
//    }
//
//    /* record the generation data */
//    DataWriter::writeBeamInstances(beamLayout.beams,  "_" + std::to_string(curr_score)+"_final_id" + std::to_string(this->_id));
//    DataWriter::writeBeamInstances(min_layout->beams, "_" + std::to_string(min_score)+ "_final_min_id" + std::to_string(this->_id));
//    DataWriter::writeGenerationSAStatistics(score_statisctis.str(),"");
//    DataWriter::writeGenerationSAStatistics(temperature_statisctis.str(),"temperature");
//    printf("finished. Final minimum score:\n");
//    layout_quality.clear();
//    Grader::overall_score_2D(*min_layout, true, layout_quality);
//    DataWriter::writeComponentGenerationParameters(this->_id, TimeLimit, ka_min, ka_max, T_min, (double)(clock()-start)/CLOCKS_PER_SEC,
//                                                   layout_quality);
//
//    /* start add to node candidates */
//    storeLayoutToFile(layer, *min_layout);
//    printf("candidates have stored to file\n");
//    cin.get();
//    this->comp_candidates = DataWriter::readComponentCandidates(this->_id);
}

void Component::SAInit(){

    //Initialize layergraph
    LayerGraph* layer = this->layerGraph;
    layer->establishAdjacentRelation(*glob_vars::sketchGraph->getLines());

    layer->pre_compute_all_placement(layer->getCurrentPlaneFlag(),
                                     vector<BeamTemplate *>());

    // put single hole beams into consideration
    layer->pre_compute_single_holes(layer->getCurrentPlaneFlag());


    layer->establishSymmetryRelation();


    skipPlaneSymmetry:

    layer->erasePlaneSymmetryRelation();
    layer->establishSymmetryUnitEdgeRelation();

    //Initialize BeamLayout
    this->beamLayout = new BeamLayout(layer->unit_edges, layer->holes, this);

    for(auto& ue : layer->unit_edges) ue->belonged_beamLayout = this->beamLayout;
    for(auto& h  : layer->holes )     h->belonged_beamLayout  = this->beamLayout;

    if(this->layerGraph->hole_num() == 1){ // special case for single hole component
        this->beamLayout->beams.clear();
        BeamInstance* single_beam = new BeamInstance(this->layerGraph->all_possi_beams.front());
        single_beam->layer_number = 0;
        this->beamLayout->beams.insert( single_beam );
    }else{
        this->beamLayout->random_init_symm();
    }

    //Initialize overall score
    OperationBeams init_optBeams;
    init_optBeams.newBeams = vector<BeamInstance*>(beamLayout->beams.begin(), beamLayout->beams.end());
    string layout_quality;
    this->gradeRecorder.updateAllInfo(*this->beamLayout, init_optBeams);

    //Initialize connection unit edges
    const double ADJACENT_THRESHOLD = 2.0 + 1e-5; //threshhold defining adjacency

    set<UnitEdge*> conn_eEdges;
    for(auto& adj_comp : adj_components){
        for(auto& h : layerGraph->holes){
            for(auto& adj_h : adj_comp->layerGraph->holes){
                if( (h->position() - adj_h->position()).norm() < ADJACENT_THRESHOLD ){

                    for(auto& ue : h->unit_edges){
                        if( std::find( layerGraph->unit_edges.begin(), layerGraph->unit_edges.end(), ue )
                                                                        != layerGraph->unit_edges.end() ){ // the adjacent unit edge is in the current component
                            conn_eEdges.insert( ue );
                            if( ue->symm_uedge != NULL )
                                conn_eEdges.insert( ue->symm_uedge );
                        }
                    }
                }
            }
        }
    }

    //TODO: haven't test here
    vector<UnitEdge*> all_edges = this->beamLayout->unit_edges;
    all_edges.erase( std::remove_if( all_edges.begin(), all_edges.end(), [conn_eEdges](UnitEdge* uEdge){
        return conn_eEdges.find(uEdge) != conn_eEdges.end();
    })
    , all_edges.end() );

}

void Component::doSimpleOperation_traditionSA(OperationBeams& oprBeams){

    /***** parameters initialization *****/
    static string layout_quality;
    oprBeams.newBeams.clear();
    oprBeams.oldBeams.clear();

    /****** Operation ******/
//    oprBeams = beamLayout->random_replace_symm();
    oprBeams = beamLayout->random_replace_symm_weighted_prob();

    this->gradeRecorder.updateAllInfo(*beamLayout, oprBeams);
}

void Component::doSimpleOperation_nonCollision(OperationBeams& oprBeams){

    /***** parameters initialization *****/
    static string layout_quality;
    oprBeams.newBeams.clear();
    oprBeams.oldBeams.clear();

    /****** Operation ******/
//    oprBeams = beamLayout->random_replace_symm();
    oprBeams = beamLayout->random_replace_symm_weighted_prob_non_collision();

    this->gradeRecorder.updateAllInfo(*beamLayout, oprBeams);
}

void Component::doSimpleOperation_unCoveredEdges(OperationBeams& oprBeams, UnitEdge* ucEdge){

    /***** parameters initialization *****/
    static string layout_quality;
    oprBeams.newBeams.clear();
    oprBeams.oldBeams.clear();

    /****** Operation ******/
    oprBeams = beamLayout->random_replace_symm_weighted_prob_non_collision_connectness( ucEdge );

    this->gradeRecorder.updateAllInfo(*beamLayout, oprBeams);

}

void Component::doConnectionOperation_SA(OperationBeams& oprBeams, vector<UnitEdge*>& consider_edges ) {

    /***** parameters initialization *****/
    static string layout_quality;
    oprBeams.newBeams.clear();
    oprBeams.oldBeams.clear();

    /****** Operation ******/
    oprBeams = beamLayout->random_replace_symm_connectivity( consider_edges );

    this->gradeRecorder.updateAllInfo(*beamLayout, oprBeams);
}
