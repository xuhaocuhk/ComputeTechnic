//
// Created by 徐豪 on 2018/9/17.
//

#include "GradeRecorder.h"

//update the corresponding data structure
void GradeRecorder::updateDeviationInfo  (BeamLayout& beamLayout, OperationBeams& operationBeams){
    //TODO : havent consider single-hole component yet
    /** old beams info **/
    int total_delete_holes_num = 0;
    int total_delele_dev       = 0;
    for(auto& b : operationBeams.oldBeams){
        total_delele_dev        += ( b->template_beam->holeNum() * ( b->layer_number * b->layer_number ) );
        total_delete_holes_num  += b->template_beam->holeNum();
    }

    /** new beams info **/
    int total_new_holes_num = 0;
    int total_new_dev       = 0;
    for(auto& b : operationBeams.newBeams){
        total_new_dev       += ( b->template_beam->holeNum() * (b->layer_number * b->layer_number) );
        total_new_holes_num += b->template_beam->holeNum();
    }

    /** update grade recorder **/
    double total_dev         = dev_old_total_dev    - total_delele_dev + total_new_dev;
    double total_hole_num    = dev_old_beamhole_num - total_delete_holes_num + total_new_holes_num;

    dev_old_total_dev = total_dev;
    dev_old_beamhole_num = total_hole_num;

}

void GradeRecorder::updateSimplicityInfo (BeamLayout& beamLayout, OperationBeams& optBeams){
    //TODO : havent consider single-hole component yet

    /** delete beams info **/
    int delete_beamhole_num = 0;
    for(auto& b : optBeams.oldBeams){
        for(auto& h : b->template_beam->covered_holes){
            beamNumOnHole[h] -= 1;
            if(beamNumOnHole[h] == 0)
                beamNumOnHole.erase(h);
        }
        delete_beamhole_num += b->template_beam->holeNum();
    }

    /** added beam info **/
    int new_beamhole_num = 0;
    for(auto& b : optBeams.newBeams){
        for(auto* h : b->template_beam->covered_holes)
            beamNumOnHole[h] += 1;
        new_beamhole_num += b->template_beam->holeNum();
    }

    /** update grader recorder **/
    int total_beamhole_num = simp_old_total_beamhole_num - delete_beamhole_num + new_beamhole_num;
    int total_coverholes   = beamNumOnHole.size();

    simp_old_total_beamhole_num = total_beamhole_num;
}

void GradeRecorder::updateRigidityInfo   (BeamLayout& beamLayout, OperationBeams& optBeams){
    //TODO : havent consider single-hole component yet

    //recover the status before perform operation
    beamLayout.undoOperation(optBeams);

    /** gradually delete beams from the original status **/
    for(auto& b : optBeams.oldBeams){
        //delete beams
        set<BeamInstance*>* current_set = beamsInRigidGroup[b];
        current_set->erase(b);
        beamLayout.eraseBeam(b);

        //re-generate rigid groups inside the current group
        while(!current_set->empty()){

            BeamInstance* beamInstance = *beamsInRigidGroup[b]->begin();
            set<BeamInstance*> rigid_component    = beamLayout.getAllRigidConnectedBeams(beamInstance);
            //make a permennant copy of the rigid component group
            set<BeamInstance*>* rigid_component_p = new set<BeamInstance*>(rigid_component);
            rigid_beam_groups.insert( rigid_component_p );

            for(auto& rb : *rigid_component_p){
                current_set->erase(rb);
                beamsInRigidGroup[rb] = rigid_component_p;
            }

        }

        rigid_beam_groups.erase(current_set);
        delete current_set;
        beamsInRigidGroup.erase(b);

        /***** update unit edge pairs ******/
        for(auto& ue : b->template_beam->covered_edges){
            curr_edges[ue] -- ;

            if(curr_edges[ue] == 0){ // the current edge are **really** be deleted

                curr_edges.erase(ue);
                adj_edges.erase(ue);

                set<UnitEdge*> adj_edges = ue->getAdjacentEdges();

                for(auto& adj_e : adj_edges){
                    if( curr_edges.find(adj_e) != curr_edges.end() )//说明当前有这个edge
                    {
                        this->adj_edges[adj_e].erase(ue); //erase the current edge from its adjacent edges
                        upairNum --;
                    }
                }
            }
        }

    }


    /** gradually add beams to the original status **/
    for(auto& b : optBeams.newBeams){

        //check rigid groups the new beam span
        set< set<BeamInstance*>* > spaned_groups;
        for(auto& u_edge : b->template_beam->covered_edges){
            for(auto& span_b : beamLayout.beamsOnEdge[u_edge]){
                spaned_groups.insert(beamsInRigidGroup[span_b]);
            }
        }

        if(spaned_groups.empty()){
            //take this single beam as a rigid component
            set<BeamInstance*> * single_group =  new set<BeamInstance*>{b};
            rigid_beam_groups.insert( single_group );
            beamsInRigidGroup[b] = single_group;

        }else{
            //merge all the connected groups and the current beam as one single group
            auto first_iter = spaned_groups.begin();
            set<BeamInstance*>* merged_group = *first_iter;
            first_iter ++;
            for(; first_iter != spaned_groups.end(); first_iter ++){
                merged_group->insert((*first_iter)->begin(), (*first_iter)->end());
                delete *first_iter;
                rigid_beam_groups.erase(*first_iter);
            }
            merged_group->insert(b);
            for(auto& new_b : *merged_group)
                beamsInRigidGroup[new_b] = merged_group;
        }

        beamLayout.addBeam(b);

        /** update unit edge pairs **/
        for(auto& ue : b->template_beam->covered_edges){
            if(curr_edges.find(ue) == curr_edges.end() ) //not exist yet
            {
                curr_edges[ue] = 1;

                set<UnitEdge*> adj_edges = ue->getAdjacentEdges();

                for(auto& adj_e : adj_edges){
                    if( curr_edges.find(adj_e) != curr_edges.end() )//说明当前有这个edge
                    {
                        this->adj_edges[adj_e].insert(ue); //erase the current edge from its adjacent edges
                        upairNum ++;
                    }
                }

            }
            else
            {
                curr_edges[ue] ++ ;
            }
        }
    }
}

void GradeRecorder::updateGapsInfo       (BeamLayout& beamLayout, OperationBeams& operationBeams){
    //TODO : havent consider single-hole component yet

    /** delete beams info **/
    for(auto& b : operationBeams.oldBeams){
        for(auto& h : b->template_beam->covered_holes){
            assert(layerNumOnHole.find(h) != layerNumOnHole.end());
            assert(std::find(layerNumOnHole[h].begin(), layerNumOnHole[h].end(), b->layer_number ) !=
                layerNumOnHole[h].end());

            //gap number before deleting this beam
            int gapNum_before = BeamLayout::gapNumber( layerNumOnHole[h] );

            //update grader recorder
            layerNumOnHole[h].erase(
                std::remove(layerNumOnHole[h].begin(),
                            layerNumOnHole[h].end(),
                            b->layer_number),
                layerNumOnHole[h].end()
            );

            int gapNum_after;
            if(layerNumOnHole[h].empty()){
                layerNumOnHole.erase(h);
                gapNum_after = 0;
            }else{
                //gap number after deleting this beam
                gapNum_after = BeamLayout::gapNumber(layerNumOnHole[h]);
            }
            gap_number = gap_number - gapNum_before + gapNum_after;
        }
    }


    /** added beams info **/
    for(auto& b : operationBeams.newBeams){
        for(auto& h : b->template_beam->covered_holes){
            int gapNum_before = 0;
            int gapNum_after  = 0;

            if(layerNumOnHole.find(h) == layerNumOnHole.end()){
                layerNumOnHole[h].push_back(b->layer_number);
                gapNum_before = 0;
                gapNum_after  = 0;
            }else{
                //gap number before adding this beam
                gapNum_before = BeamLayout::gapNumber(layerNumOnHole[h]);

                //update grader recorder
                layerNumOnHole[h].push_back(b->layer_number);

                //gap number after adding this beam
                gapNum_after  = BeamLayout::gapNumber(layerNumOnHole[h]);
            }
            gap_number = gap_number - gapNum_before + gapNum_after;
        }
    }
}

void GradeRecorder::updateBoundInfo      (BeamLayout& beamLayout, OperationBeams& operationBeams){
    //TODO : havent consider single-hole component yet
    /** delete beams info **/
    for(auto& b : operationBeams.oldBeams){
        beamsSortedByLayer.erase(b);
    }

    /** added beams info **/
    for(auto& b : operationBeams.newBeams){
        beamsSortedByLayer.insert(b);
    }
}

void GradeRecorder::updateSymmetryInfo   (BeamLayout& beamLayout, OperationBeams& operationBeams){
    //TODO : havent consider single-hole component yet
    /** delete beams info **/
    for(auto& b : operationBeams.oldBeams){
        if(b->symm_beam != NULL){
            symm_layer_deviation -= (b->layer_number - b->symm_beam->layer_number) *
                                             (b->layer_number - b->symm_beam->layer_number);
        }
    }

    /** added beams info **/
    for(auto& b : operationBeams.newBeams){
        if(b->symm_beam != NULL){
            symm_layer_deviation += (b->layer_number - b->symm_beam->layer_number) *
                                             (b->layer_number - b->symm_beam->layer_number);
        }
    }
}

void GradeRecorder::updateCompleteInfo   (BeamLayout& beamLayout, OperationBeams& operationBeams){
    //TODO : havent consider single-hole component yet
    // do nothing
}

void GradeRecorder::updateAllInfo        (BeamLayout& beamLayout, OperationBeams& operationBeams){
    //TODO : havent consider single-hole component yet
    updateDeviationInfo(beamLayout, operationBeams);
    updateSimplicityInfo(beamLayout, operationBeams);
    updateRigidityInfo(beamLayout, operationBeams);
    updateGapsInfo(beamLayout, operationBeams);
    updateBoundInfo(beamLayout, operationBeams);
    updateSymmetryInfo(beamLayout, operationBeams);
    updateCompleteInfo(beamLayout, operationBeams);
}