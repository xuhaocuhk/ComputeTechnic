//
// Created by 徐豪 on 2018/6/25.
//

#include "UnitEdge.h"
#include "configure/global_variables.h"
#include "LayerGraph.h"
#include "configure/Constants.h"


int UnitEdge::max_id = 0;


UnitEdge::UnitEdge(SketchLine* parent, LayerHole* hole1, LayerHole* hole2)
{
    this->_id = max_id ++;
    this->parentline = parent;
    assert( fabs((hole1->position() - hole2->position()).norm() - 1) <0.01 );
    this->hole1 = hole1;
    this->hole2 = hole2;
}

set<UnitEdge*> UnitEdge::getAdjacentEdges()
{
    set<UnitEdge*> adjEdges;
    for(auto& ue : hole1->unit_edges){
        if(ue != this) adjEdges.insert(ue);
    }
    for(auto& ue : hole2->unit_edges){
        if(ue != this) adjEdges.insert(ue);
    }
    return adjEdges;
}

set<UnitEdge*> UnitEdge::getAdjacentEdges(const set<UnitEdge*>& given_set)//find adjacent edges inside a given set
{
    set<UnitEdge*> adjEdges;
    for(auto& ue : hole1->unit_edges){
        if(ue != this && given_set.find(ue) != given_set.end())
            adjEdges.insert(ue);
    }
    for(auto& ue : hole2->unit_edges){
        if(ue != this && given_set.find(ue) != given_set.end())
            adjEdges.insert(ue);
    }
    return adjEdges;
}

bool UnitEdge::isAdjacent(UnitEdge* edge)
{
    assert(this != edge);
    set<UnitEdge*> adj_edges = this->getAdjacentEdges();
    if(adj_edges.find(edge) != adj_edges.end())
        return true;
    return false;
}

void DFS_symmetry_uEdges(UnitEdge* uedge, set<UnitEdge*>& result){
    result.insert(uedge);
    set<UnitEdge*> adj_groups;
    if(uedge->symm_uedge_x != NULL) adj_groups.insert(uedge->symm_uedge_x);
    if(uedge->symm_uedge_y != NULL) adj_groups.insert(uedge->symm_uedge_y);
    if(uedge->symm_uedge_z != NULL) adj_groups.insert(uedge->symm_uedge_z);
    for(auto& e : adj_groups){
        if(result.find(e) == result.end())//1.需要在这个component上 2)rigid 3)当前数据集中还没有
            DFS_symmetry_uEdges(e,result);
    }
}

set<UnitEdge*> UnitEdge::getAllSymmetryUnitEdges()//得到当前group的所有symmetry groups，包括自己
{
    set<UnitEdge*> result;
    DFS_symmetry_uEdges(this,result);
    return result;
}

void DFS_connect_edges(UnitEdge* unitEdge, set<UnitEdge*, UnitEdge::UnitEdgeIDComp>& result, const set<UnitEdge*>& consider_edges){
    result.insert(unitEdge);
    set<UnitEdge*> adj_edges = unitEdge->getAdjacentEdges(consider_edges);

    for(auto& adj_ue : adj_edges){
        if( result.find(adj_ue) == result.end() )
            DFS_connect_edges(adj_ue, result, consider_edges);
    }
}

set<UnitEdge*, UnitEdge::UnitEdgeIDComp> UnitEdge::getAllConnectEdgesInGroup(const set<UnitEdge*>& consider_set){
    set<UnitEdge*, UnitEdge::UnitEdgeIDComp> result;
    DFS_connect_edges(this, result, consider_set);
    return result;
}

/*
 * 返回两条线的所有可能的关于坐标轴对称的方向,如果是空集说明不对称
 */
set<int> UnitEdge::symmetryDirections(UnitEdge *line){
    set<int> symmetry_direction;

    if(glob_vars::current_model.symmetry_x && this->symm_uedge_x == line ){
        symmetry_direction.insert(Constants::X_AXIS);
    }

    if(glob_vars::current_model.name == "bridge"){}
    else{
        if(glob_vars::current_model.symmetry_y && this->symm_uedge_y == line ){
            symmetry_direction.insert(Constants::Y_AXIS);
        }
    }

    if(glob_vars::current_model.symmetry_z && this->symm_uedge_z == line ){
        symmetry_direction.insert(Constants::Z_AXIS);
    }
    return symmetry_direction;
}
