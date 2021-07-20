//
// Created by 徐豪 on 2018/9/14.
//

#ifndef LEGO_TECHNIC_MAIN_GRADERECORDER_H
#define LEGO_TECHNIC_MAIN_GRADERECORDER_H

#include <bricks/BeamInstance.h>
#include <map>
#include "core/BeamLayout.h"

using std::map;
struct GradeRecorder{ // record the last calculatio status for online update score

    /** deviation **/
    double dev_old_total_dev = 0.0;
    int    dev_old_beamhole_num = 0;

    /** simplicity **/
    int    simp_old_total_beamhole_num = 0.0;
    map<LayerHole*, int> beamNumOnHole; //mapping from beamhole to number of beams on it

    /** rigidity **/
    set< set<BeamInstance*>* > rigid_beam_groups;
    map<BeamInstance*, set<BeamInstance*>* > beamsInRigidGroup;
    //unit pairs
    int upairNum = 0;
    map< UnitEdge*, set<UnitEdge*> >  adj_edges;
    map< UnitEdge*, int            >  curr_edges; //TODO:change to unorderd map here

    /** gap **/
    int gap_number = 0;
    map<LayerHole*, vector<int>> layerNumOnHole;

    /** bound **/
    struct BeamLayerComp {
        bool operator() (BeamInstance* b1, BeamInstance* b2) const
        {
            if(b1->layer_number < b2->layer_number){
                return true;
            }else if(b1->layer_number > b2->layer_number){
                return false;
            }else{ //should be distinguished when the beam layer numbers are equal
                return b1 < b2;
            }
        }
    };
    set<BeamInstance*, BeamLayerComp> beamsSortedByLayer;

    /** symmetry **/
    int symm_layer_deviation = 0;



    //update the corresponding data structure
    void updateDeviationInfo  (BeamLayout&, OperationBeams&);
    void updateSimplicityInfo (BeamLayout&, OperationBeams&);
    void updateRigidityInfo   (BeamLayout&, OperationBeams&);
    void updateGapsInfo       (BeamLayout&, OperationBeams&);
    void updateBoundInfo      (BeamLayout&, OperationBeams&);
    void updateSymmetryInfo   (BeamLayout&, OperationBeams&);
    void updateCompleteInfo   (BeamLayout&, OperationBeams&);
    void updateAllInfo        (BeamLayout&, OperationBeams&);

};


#endif //LEGO_TECHNIC_MAIN_GRADERECORDER_H
