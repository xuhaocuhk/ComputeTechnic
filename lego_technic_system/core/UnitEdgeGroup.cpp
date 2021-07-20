//
// Created by 徐豪 on 2018-10-18.
//

#include "UnitEdgeGroup.h"
#include "LayerGraph.h"
#include "configure/global_variables.h"
#include "util/util.h"
#include <fstream>
#include <zconf.h>
#include <util/Symmetry.h>

using std::tuple;


UnitEdgeGroup::UnitEdgeGroup( set<UnitEdge*, UnitEdge::UnitEdgeIDComp> & group_uedges):
u_edges(group_uedges)
{

}


void DFS_symmetry_groups_with_dirc( tuple<UnitEdgeGroup*, vector<int>> & g, set<tuple<UnitEdgeGroup*, vector<int>>>& result, UnitEdgeGroup* current_group){

    if(std::get<0>(g) != current_group){
        result.insert(g);
    }
    vector<int> last_direction = std::get<1>(g);
    set<tuple<UnitEdgeGroup*,vector<int>>> adj_groups;

    if(std::get<0>(g)->symm_group_x != NULL){
        vector<int> current_direc = last_direction;
        current_direc.push_back(Constants::X_AXIS);
        adj_groups.insert(std::make_tuple(std::get<0>(g)->symm_group_x, current_direc));
    }
    if(std::get<0>(g)->symm_group_y != NULL){
        vector<int> current_direc = last_direction;
        current_direc.push_back(Constants::Y_AXIS);
        adj_groups.insert(std::make_tuple(std::get<0>(g)->symm_group_y, current_direc));
    }
    if(std::get<0>(g)->symm_group_z != NULL){
        vector<int> current_direc = last_direction;
        current_direc.push_back(Constants::Z_AXIS);
        adj_groups.insert(std::make_tuple(std::get<0>(g)->symm_group_z, current_direc));
    }
    for(auto e : adj_groups){
        if(std::find_if(result.begin(),result.end(), [&e](const tuple<UnitEdgeGroup*, vector<int>>& tuple1)
        { return std::get<0>(tuple1) == std::get<0>(e); })
           == result.end() && std::get<0>(e) != current_group)
            DFS_symmetry_groups_with_dirc(e, result,current_group);
    }
}


set<std::tuple<UnitEdgeGroup*, vector<int>>> UnitEdgeGroup::getAllSymmetryUnitEdgeGroupsWithDirecTag(){
    set<tuple<UnitEdgeGroup*,vector<int>>> result;
    tuple<UnitEdgeGroup*,vector<int>> this_tuple = std::make_tuple(this,vector<int>());
    DFS_symmetry_groups_with_dirc(this_tuple, result, this);
    return result;
}


set<int> UnitEdgeGroup::symmetryDirections(UnitEdgeGroup *group)//return all the symmetry direction between this two groups
{
    set<int> symmetryDirections{Constants::X_AXIS, Constants::Y_AXIS, Constants::Z_AXIS};

    if( this->u_edges.size() != group->u_edges.size() )
        return set<int>();

    for(auto& line1 : this->u_edges){
        set<int> symm_direction_line1;

        for(auto& line2 : group->u_edges){
            auto lineSymmDirections = line1->symmetryDirections(line2);

            if(!lineSymmDirections.empty()){
                symm_direction_line1 = lineSymmDirections;
                break;
            }

        }

        if(symm_direction_line1.empty())
            return symm_direction_line1;
        else{
            set<int> result;

            std::set_intersection(symmetryDirections.begin(), symmetryDirections.end(),
                                  symm_direction_line1.begin(),symm_direction_line1.end(),
                                  std::inserter(result,result.end()));
            symmetryDirections = result;

        }
    }

    return symmetryDirections;
}

void UnitEdgeGroup::establishSymmetryGroupRelation(vector<UnitEdgeGroup *> &groups){

    for(auto group1 = groups.begin(); group1 != groups.end(); group1++){
        for(auto group2 = group1 +1; group2 != groups.end(); group2++){

            set<int> symmDirections = (*group1)->symmetryDirections(*group2);

            if(!symmDirections.empty()){
//                if(!(*group1)->symmetryGroups.empty()){
//                    printf("more than one symmetry group!!!\n");
//                    for(auto& d :(*group1)->symmetryGroups) printf("%d\n",std::get<1>(d));
//                    cin.get();
//                }

                if(glob_vars::current_model.name == "tokyo_temple"){}
                else{
                    if(symmDirections.find(Constants::X_AXIS) != symmDirections.end() && glob_vars::current_model.symmetry_x){
                        (*group1)->symm_group_x = (*group2);
                        (*group2)->symm_group_x = (*group1);
                    }
                }
                if(symmDirections.find(Constants::Y_AXIS) != symmDirections.end() && glob_vars::current_model.symmetry_y){
                    (*group1)->symm_group_y = (*group2);
                    (*group2)->symm_group_y = (*group1);
                }
                if(symmDirections.find(Constants::Z_AXIS) != symmDirections.end() && glob_vars::current_model.symmetry_z){
                    (*group1)->symm_group_z = (*group2);
                    (*group2)->symm_group_z = (*group1);
                } else{
//                    printf("这什么玩意\n");
                }
            }
        }
    }
}

vector<Connector*> UnitEdgeGroup::getSymmetryConnectors(const set<Connector*>& m_conns){
    auto symm_groups = this->getAllSymmetryUnitEdgeGroupsWithDirecTag();
    assert(!symm_groups.empty());
    vector<Connector*> result;
    for(auto& group : symm_groups){
        vector<Connector*> result_group(m_conns.begin(), m_conns.end());

        vector<int> symm_directions = std::get<1>(group);
        for(int direction : symm_directions){
            result_group = Symmetry::get_all_reflect_conns_general(result_group, direction);
        }

        result.insert(result.end(), result_group.begin(), result_group.end());
    }
    return result;
}

/*
 * 1.No collision among connectors
 * 2.covers all beams inside this group
 */
bool valid_connectors(const vector<Connector>& chosen_conns, set<UnitEdge*,  UnitEdge::UnitEdgeIDComp>& group){

    /** no collision among connectors **/
    for(int i = 0;i<chosen_conns.size(); i++){
        for(int j = i+1; j<chosen_conns.size(); j++){
            if(chosen_conns.at(i).collision( chosen_conns.at(j) ))
                return false;
        }
    }

    /** covers all edges in group **/
    set<UnitEdge*> total_covered_edges;
    for(auto& c : chosen_conns){
        total_covered_edges.insert(c.covered_edges.begin(), c.covered_edges.end());
    }

    if(total_covered_edges.size() != group.size()){
        return false;
    }

    return true;

}

void select_conns(const vector<int> &v, const vector<Connector>& all_conns,
                  vector<vector<Connector>>& selected_cons, set<UnitEdge*,  UnitEdge::UnitEdgeIDComp>& group ) {
    vector< Connector > current_con;

    for (int i = 0; i < v.size(); ++i) {
        current_con.push_back(all_conns.at(v[i]));
    }

    if( valid_connectors(current_con, group) )
        selected_cons.push_back(current_con);
}

//try all combination select k items from all connectors
vector<int> combinationn;
void all_combination(int N, int offset, int k, const vector<Connector>& all_conns,
                     vector<vector<Connector>>& selected_cons, set<UnitEdge*,  UnitEdge::UnitEdgeIDComp>& group) {
    if (k == 0) {
        select_conns(combinationn, all_conns, selected_cons, group);
        return;
    }
    for (int i = offset; i <= N - k; ++i) {
        combinationn.push_back(i);
        all_combination(N, i + 1, k - 1, all_conns, selected_cons, group);
        combinationn.pop_back();
    }
}


bool UnitEdgeGroup::genConnectorsForThisGroup(BeamLayoutConnectorGenerator & connGenerator){

    /***** collecting all related beamholes *****/
    set<BeamHole*> related_beamholes;

    for(auto& ue : this->u_edges){
        this->related_layerholes.insert(ue->hole1);
        this->related_layerholes.insert(ue->hole2);
    }

    ///// Enroll one-ring neibor into consideration
    set<LayerHole*> neibor_holes;
    for(auto& h : this->related_layerholes){
        for(auto& ue : h->unit_edges){
            neibor_holes.insert(ue->hole1);
            neibor_holes.insert(ue->hole2);
        }
    }
    this->related_layerholes.insert( neibor_holes.begin(), neibor_holes.end() );

    for(auto& layer_hole : this->related_layerholes){
        for(auto& bh : connGenerator.beamholesOnLayerHole[layer_hole]){
            if( connGenerator.connectorsOnHole[bh].empty() ){
                related_beamholes.insert(bh);
            }
        }
    }
    ////remove holes who have already assigned connectors


    /***** exaustively find all possible beams inside this group *****/

    auto all_connectors_for_group = connGenerator.findAllPossibleConnector(related_beamholes);

    /**** Establish edge cover information ****/
    for(auto& c : all_connectors_for_group){
        for(auto & ue : this->u_edges){
            if(c.covers(ue)){
                c.covered_edges.push_back(ue);
            }
        }
    }

    //// debugging.... show covered edges
//        for(auto& c : all_connectors_for_group){
//            c.toLDrawFormat();
//            for(auto& ue : c.covered_edges){
//                ue->hole1->toLdrawFormat();
//                ue->hole2->toLdrawFormat();
//            }
//            printf("-----\n");
//        }

    all_connectors_for_group.erase( std::remove_if(all_connectors_for_group.begin(), all_connectors_for_group.end(),
                                                   [](const Connector& c) {
                                                       return c.covered_edges.empty(); //remove connectors covers no edges
                                                   }), all_connectors_for_group.end());

    std::sort(all_connectors_for_group.begin(),all_connectors_for_group.end(),[](const Connector& c1, const Connector& c2) -> bool
    {
        return c1.covered_edges.size() > c2.covered_edges.size();
    });

    ///// Exaustively choose connectors to cover edges
    vector<vector<Connector>> best_choices;
    vector<Connector> chosen_connectors;
    for(int select_conn_num = 1; select_conn_num <= all_connectors_for_group.size(); select_conn_num++){

        all_combination(all_connectors_for_group.size(), 0, select_conn_num, all_connectors_for_group, best_choices, this->u_edges);

        if(best_choices.size()==0){

            continue;
        }else if(best_choices.size()==1){
            chosen_connectors = best_choices.front();
            break;
        }else{//more than one valid connectors
            double max_score = -9999;
            int best_index = 0;
            for(int i = 0; i<best_choices.size(); i++){
                double current_score = Grader::pinhead_ratio(best_choices.at(i));
                if(current_score > max_score){
                    best_index = i;
                    max_score = current_score;
                }
            }
            chosen_connectors = best_choices[best_index];
            break;
        }
    }

    if(chosen_connectors.empty()){
        return false;
    }else{
        for(auto& c : chosen_connectors){
            Connector * c_pt = new Connector(c);
            connGenerator.connectors.insert(c_pt);
            this->assigned_connectors.insert(c_pt);
            for(auto& bh_c : c.connected_holes){
                connGenerator.connectorsOnHole[bh_c].push_back(c_pt);
            }
        }
        return true;
    }

}