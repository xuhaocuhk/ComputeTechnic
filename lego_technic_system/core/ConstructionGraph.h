//
// Created by 徐豪 on 2018/5/19.
//

#ifndef LEGOGENERATION_MAIN_CONNECTIONGRAPH_H
#define LEGOGENERATION_MAIN_CONNECTIONGRAPH_H


#include "Component.h"
#include <vector>
#include "connector/BeamLayoutConnectorGenerator.h"

using std::tuple;
using std::get;
using std::vector;


class Component;
class BeamLayoutConnectorGenerator;


class ConstructionGraph {

public:
    ConstructionGraph(vector<LayerGraph*>&);
    Component* getCenterNode();//得到拓扑意义上的中心节点

    /**** SA Initialization ****/
    void initAffectedHoles();


    /**** Generating beams by SA ****/
    void genComponentCandidates3D_baseline();//using a 3D SA to generate the whole result
    void genComponentCandidates3D_weighted_picking();
    void genComponentCanndidate3D_weighted_picking_adptiveSchedul(); // Not very effective
    void genComponentCandidates3D_weighted_picking_non_collision(); // always keep no collision among all components during SA
    void genComponentCandidates3D_weighted_picking_non_collision_withConnector(); //enroll connector into consideration
    void genComponentCandidates3D_weighted_picking_non_collision_withConnector_Phase2(); //enroll connector into consideration

    /**** Second Step processing ****/
    void tryTwoOrientForMinorComponents( );


    /**** After SA for beams, delete unit edges | components | beams that collide with others ****/
    void deleteCollideElements();

    void exaustiveGeneration();

    vector<vector<Component*>> groupingByAdjacent(set<Component*>&);

    void printConnectionRelation();
    void printComponentContactingNum();
    void writeMajorComponents();

public:
    vector<Component*> components;
    Component* center_node = NULL;
    BeamLayoutConnectorGenerator* beamLayoutConnectorGenerator = NULL;
};


#endif //LEGOGENERATION_MAIN_CONNECTIONGRAPH_H
