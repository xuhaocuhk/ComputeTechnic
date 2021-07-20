//
// Created by 徐豪 on 2018-10-18.
//

#ifndef LEGO_TECHNIC_MAIN_UNITEDGEGROUP_H
#define LEGO_TECHNIC_MAIN_UNITEDGEGROUP_H


#include "UnitEdge.h"

class BeamLayoutConnectorGenerator;

class UnitEdgeGroup {

public:
    struct UnitEdgeGroupComp {
        bool operator() (UnitEdgeGroup* g1, UnitEdgeGroup* g2) const
        {
            if( (*g1->u_edges.begin())->_id > (*g2->u_edges.begin())->_id ){
                return false;
            }else if ( (*g1->u_edges.begin())->_id < (*g2->u_edges.begin())->_id ){
                return true;
            }else{
                return (*g1->related_layerholes.begin())->_id < (*g2->related_layerholes.begin())->_id;
            }
        }
    };

public:
    UnitEdgeGroup( set<UnitEdge*, UnitEdge::UnitEdgeIDComp>& );

    /******* Symmetry Groups *******/
    set<std::tuple<UnitEdgeGroup*, vector<int>>> getAllSymmetryUnitEdgeGroupsWithDirecTag();

    set<int> symmetryDirections(UnitEdgeGroup *group);//all the symmetry direction between two groups

    static void establishSymmetryGroupRelation(vector<UnitEdgeGroup *> &groups);

    vector<Connector*> getSymmetryConnectors(const set<Connector*>& m_conns);


    /****** Generation ******/
    bool genConnectorsForThisGroup(BeamLayoutConnectorGenerator&);


public:

    set<LayerHole*> related_layerholes;

    set<UnitEdge*, UnitEdge::UnitEdgeIDComp> u_edges;

    UnitEdgeGroup* symm_group_x = NULL;
    UnitEdgeGroup* symm_group_y = NULL;
    UnitEdgeGroup* symm_group_z = NULL;

    set<Connector*> assigned_connectors;
};


#endif //LEGO_TECHNIC_MAIN_UNITEDGEGROUP_H
