//
// Created by XuHao on 4/24/2019.
//

#ifndef LEGO_TECHNIC_MAIN_GUIDINGGRAPH_H
#define LEGO_TECHNIC_MAIN_GUIDINGGRAPH_H


#include "UnitEdge.h"
#include "LayerHole.h"
#include "core/SketchGraph.h"

class GuidingGraph {

public:
    GuidingGraph(SketchGraph*);
    void establishLayerHoleSymmetry();
    void establishUnitEdgeSymmetry();

public:
    set<UnitEdge*  , UnitEdge::UnitEdgeIDComp > u_edges;
    set<LayerHole* , LayerHole::LayerHoleComp > m_holes;
};


#endif //LEGO_TECHNIC_MAIN_GUIDINGGRAPH_H
