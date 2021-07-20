//
// Created by 徐豪 on 2017/11/13.
//

#include <list>
#include "BeamHole.h"

BeamHole::BeamHole(Vector3d pos, bool isAxle, Vector3d normal, int layer_num):
pos(pos),isAxle(isAxle),normal(normal),layer_number_on_beam(layer_num)
{
    assigned_connector = 0;
}

//得到通过possible_connectors连接的所有的BeamHoles
set<BeamHole*> BeamHole::getPossiConnectedBeamHoles()
{
    set<BeamHole*> result;
    for(const auto & c : this->possi_connectors){
        for(const auto & bh : c->connected_holes){
            if(bh != this)
                result.insert(bh);
        }
    }
    return result;
}

set<BeamHole*> genUnvisitedBeamHoles(BeamHole* bh, const set<BeamHole*>& m_result){
    set<BeamHole*> result;
    for(const auto & hhh : bh->getPossiConnectedBeamHoles()){
        if (m_result.find(hhh) == m_result.end()){
            result.insert(hhh);
        }
    }
    return result;
}

void DFS(BeamHole *bh, set<BeamHole *> &result)//DFS search unconnected beam holes
{
    result.insert(bh);//means visited

    set<BeamHole*> adj_set = genUnvisitedBeamHoles(bh, result);
    for(BeamHole* adj_bh : adj_set){
        DFS(adj_bh, result);
    }
}

void BFS_connected_holes(BeamHole *bh, set<BeamHole *> &result, int number)//DFS search unconnected beam holes, return if the size is enough
{

    // Create a queue for BFS
    std::list<BeamHole*> queue;

    queue.push_back(bh);
    result.insert(bh);

    while(!queue.empty() && result.size()<number)
    {
        // Dequeue a vertex from queue and print it
        bh = queue.front();
        queue.pop_front();
        set<BeamHole*> adj_set = genUnvisitedBeamHoles(bh, result);

        for (auto i = adj_set.begin(); i != adj_set.end(); ++i)
        {
            queue.push_back(*i);
            result.insert(*i);
        }
    }
}


set<BeamHole*> BeamHole::getAllPotentialConnectedBeamHoles()//得到所有间接通过possible_connectors连接的所有的BeamHoles,包括自己
{
    set<BeamHole*> result;
    DFS(this, result);
    return result;
}
//上一个方法的加数量限制的版本
set<BeamHole*> BeamHole::getAllPotentialConnectedBeamHoles(int number)
{
    set<BeamHole*> result;
    BFS_connected_holes(this, result, number);
    return result;
}

bool BeamHole::alreadyBeenInserted()
{
    return this->assigned_connector!=0;
}

void BeamHole::toLDrawFormat(int color){
    //根据normal计算rotation矩阵
    Vector3d v1 = Vector3d(0,1,0);
    Vector3d cross_v1_v2 = v1.cross(this->normal);
    Matrix3d rot_mat;
    if(cross_v1_v2.norm() == 0)
        rot_mat = Matrix3d::Identity(3,3);
    else{
        rot_mat = Eigen::AngleAxisd(acos(v1.dot(this->normal)/(v1.norm()* this->normal.norm())), cross_v1_v2/cross_v1_v2.norm()).matrix();
    }
    printf("1 %d %lf %lf %lf %lf  %lf %lf %lf %lf %lf %lf %lf %lf 18654.dat\n", color, this->pos.x()*20, this->pos.y()*20, this->pos.z() * 20,
           rot_mat(0,0),rot_mat(0,1),rot_mat(0,2),
           rot_mat(1,0),rot_mat(1,1),rot_mat(1,2),
           rot_mat(2,0),rot_mat(2,1),rot_mat(2,2));
}

string BeamHole::toLDrawFormatAsString(int color){
    //根据normal计算rotation矩阵
    Vector3d v1 = Vector3d(0,1,0);
    Vector3d cross_v1_v2 = v1.cross(this->normal);
    Matrix3d rot_mat;
    if(cross_v1_v2.norm() == 0)
        rot_mat = Matrix3d::Identity(3,3);
    else{
        rot_mat = Eigen::AngleAxisd(acos(v1.dot(this->normal)/(v1.norm()* this->normal.norm())), cross_v1_v2/cross_v1_v2.norm()).matrix();
    }

    char buff[200];
    snprintf(buff, sizeof(buff), "1 %d %lf %lf %lf %lf  %lf %lf %lf %lf %lf %lf %lf %lf 18654.dat\n", color, this->pos.x()*20, this->pos.y()*20, this->pos.z() * 20,
           rot_mat(0,0),rot_mat(0,1),rot_mat(0,2),
           rot_mat(1,0),rot_mat(1,1),rot_mat(1,2),
           rot_mat(2,0),rot_mat(2,1),rot_mat(2,2));

    std::string buffAsStdStr = buff;
    return buffAsStdStr;
}