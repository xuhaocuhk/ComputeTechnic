//
// Created by 徐豪 on 2018/5/19.
//

#ifndef LEGOGENERATION_MAIN_LAYERNODE_H
#define LEGOGENERATION_MAIN_LAYERNODE_H

#include "LayerGraph.h"
#include "BeamLayout.h"
#include "objectives/Grader.h"
#include "objectives/GradeRecorder.h"
#include "UnitEdgeGroup.h"


//保存有当前component的所有信息
class Component {

public:

    /***** Algorithm initialization *****/
    void SAInit();

    /***** SA operations *****/
    void doSimpleOperation_traditionSA(OperationBeams&);
    void doSimpleOperation_nonCollision(OperationBeams&);
    void doSimpleOperation_unCoveredEdges(OperationBeams&, UnitEdge* );
    void doConnectionOperation_SA(OperationBeams&, vector<UnitEdge*>& ); //operation in phase two, to make the structure connectable

    /***** SA algorithms*****/
    void gen2DComponentCandidates_TSA_adaptiveK();//For code reading only, the quality/time tradeoff k will vary with time remains

public:

    /***** Data stored for SA *****/
    BeamLayout* beamLayout     = NULL;
    BeamLayout* minLayout      = NULL;
    GradeRecorder gradeRecorder;
    std::ostringstream score_statisctis;
    std::ostringstream temperature_statisctis;


    /***** Basic Data Structure *****/
    int _id = -1;
    vector<Component*> adj_components;
    LayerGraph* layerGraph = NULL;//当前node中的graph

};


#endif //LEGOGENERATION_MAIN_LAYERNODE_H
