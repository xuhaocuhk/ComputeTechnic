//
// Created by 徐豪 on 2018/6/25.
//

#ifndef LEGO_TECHNIC_MAIN_UNITEDGE_H
#define LEGO_TECHNIC_MAIN_UNITEDGE_H


#include "LayerHole.h"
#include <vector>

class SketchLine;
class LayerHole;

class BeamTemplate;
class BeamLayout;

using std::vector;
using std::set;


class UnitEdge {

public:
    struct UnitEdgeIDComp {
        bool operator() (UnitEdge* u1, UnitEdge* u2) const
        {
            return u1->_id > u2->_id;
        }
    };


public:
    int _id;
    static int max_id;
    LayerHole* hole1 = NULL;
    LayerHole* hole2 = NULL;
    SketchLine* parentline = NULL;
    vector<BeamTemplate *> possibleBeams;
    UnitEdge* symm_uedge = NULL; // unit edges inside a component
    UnitEdge* symm_uedge_x = NULL;
    UnitEdge* symm_uedge_y = NULL;
    UnitEdge* symm_uedge_z = NULL;
    BeamLayout* belonged_beamLayout = NULL;

public:
    UnitEdge(SketchLine* parent, LayerHole* hole1, LayerHole* hole2);
    set<UnitEdge*> getAdjacentEdges();
    set<UnitEdge*> getAdjacentEdges(const set<UnitEdge*>&);//find adjacent edges inside a given set
    bool isAdjacent(UnitEdge*);

    set<UnitEdge*> getAllSymmetryUnitEdges();//得到当前group的所有symmetry groups，包括自己

    set<UnitEdge*, UnitEdge::UnitEdgeIDComp> getAllConnectEdgesInGroup(const set<UnitEdge*>& ); //得到所有的传递相连的unitedges， 包括自己

    set<int> symmetryDirections(UnitEdge *line);
};


#endif //LEGO_TECHNIC_MAIN_UNITEDGE_H
