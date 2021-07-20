//
// Created by 徐豪 on 2017/8/1.
//
#include <iostream>
#include <set>
#include <util/Helper.h>
#include "util/util.h"
#include "LayerGraph.h"
#include "configure/global_variables.h"
#include "SketchLine.h"
int SketchLine::max_id = 0;

SketchLine::SketchLine()
{

}

SketchLine::SketchLine(SketchPoint *p1, SketchPoint *p2, bool is_feature_line):_p1(p1), _p2(p2){
    _id = max_id++;
    p1->addLine(this);
    p2->addLine(this);
    if(!integer_length_check(p1,p2)){
        printf("Error!! Non integet length line!!!\n");
        integer_length_check(p1,p2);
        std::cout<<p1->getPointVec()<<std::endl;
        std::cout<<p2->getPointVec()<<std::endl;
        std::cout<<(p1->getPointVec()-p2->getPointVec()).norm()<<endl;
//        exit(0);
    }
    this->feature_line = is_feature_line;
}

Vector3d SketchLine:: center(){
    double center_x = (_p1->_x+_p2->_x)/2;
    double center_y = (_p1->_y+_p2->_y)/2;
    double center_z = (_p1->_z+_p2->_z)/2;
    return Vector3d(center_x,center_y,center_z);
}

double SketchLine::length(){
    return sqrt((_p1->_x-_p2->_x)*(_p1->_x-_p2->_x)+(_p1->_y-_p2->_y)*(_p1->_y-_p2->_y)+(_p1->_z-_p2->_z)*(_p1->_z-_p2->_z));
}

int SketchLine::intLength(){
    assert( fabs(length() - round(length())) < 5e-2);
    return (int)round(length());
}

Eigen::Vector3d SketchLine::getVector(){
    return Eigen::Vector3d(_p1->_x-_p2->_x,_p1->_y-_p2->_y,_p1->_z-_p2->_z);
}

set<SketchLine*> SketchLine::getAdjLines(){
    std::set<SketchLine*> result;
    result.insert(this->_p1->lines.begin(),this->_p1->lines.end());
    result.insert(this->_p2->lines.begin(),this->_p2->lines.end());
    result.erase(this);
    return result;
}


void DFS(SketchLine* line, set<SketchLine*>& result, set<SketchLine*> consider_lines){
    result.insert(line);
    set<SketchLine*> adj_lines = line->getAdjLines();
    for(auto it = adj_lines.begin(); it != adj_lines.end(); )
        if(((result.find(*it) != result.end()))
           ||(consider_lines.find(*it) == consider_lines.end()))
            it = adj_lines.erase(it);
        else
            ++it;
    for(auto& l : adj_lines)
        DFS(l,result,consider_lines);
}

void DFS(SketchLine* line, set<SketchLine*>& result, int line_tag, int line_tag1, int line_tag2, set<SketchLine*> consider_lines){
    result.insert(line);
    set<SketchLine*> adj_lines = line->getAdjLines();
    for(auto it = adj_lines.begin(); it != adj_lines.end();)
        if(((*it)->possi_orientation != line_tag1 && (*it)->possi_orientation != line_tag2 && (*it)->possi_orientation != line_tag)
           || (result.find(*it) != result.end())
           || (consider_lines.find(*it) == consider_lines.end()))
            it = adj_lines.erase(it);
        else
            ++it;
    for(auto& l : adj_lines)
        DFS(l, result, line_tag, line_tag1, line_tag2, consider_lines);
}

void DFS(SketchLine* line, set<SketchLine*>& result, int line_tag, set<SketchLine*> consider_lines){
    result.insert(line);
    set<SketchLine*> adj_lines = line->getAdjLines();
    for(auto it = adj_lines.begin(); it != adj_lines.end();)
        if((  (*it)->chosen_orientation != line_tag
           || (*it)->containsDiffOrient(line))
           || !line->containsSamePossiOrient(*it)
           || (result.find(*it) != result.end())
           || (consider_lines.find(*it) == consider_lines.end()))
            it = adj_lines.erase(it);
        else
            ++it;
    for(auto& l : adj_lines)
        DFS(l, result, line_tag, consider_lines);
}


set<SketchLine*> SketchLine::getAllAdjLines(set<SketchLine*> consider_lines)//返回所有邻接的非major plane的lines
{
    std::set<SketchLine*> result;
    DFS(this,result,consider_lines);
    return result;
}

set<SketchLine*> SketchLine::getAllAdjLinesByPossiOrient(set<SketchLine *> consider_lines, int line_tag)
{
    std::set<SketchLine*> result;
    int line_tag1;
    int line_tag2;
    if(line_tag==Constants::PLANE_XZ){
        line_tag1 = Constants::PLANE_YZ_OR_XZ;
        line_tag2 = Constants::PLANE_XY_OR_XZ;
    }else if(line_tag == Constants::PLANE_YZ){
        line_tag1 = Constants::PLANE_YZ_OR_XZ;
        line_tag2 = Constants::PLANE_XY_OR_YZ;
    }else if(line_tag == Constants::PLANE_XY){
        line_tag1 = Constants::PLANE_XY_OR_YZ;
        line_tag2 = Constants::PLANE_XY_OR_XZ;
    }
    DFS(this,result,line_tag,line_tag1,line_tag2, consider_lines);
    return result;
}

set<SketchLine*> SketchLine::getAllAdjLinesByChosenOrient(set<SketchLine *> consider_lines, int line_tag)
{
    std::set<SketchLine*> result;
    DFS(this, result, line_tag, consider_lines);
    return result;
}

SketchPoint* SketchLine::intersectPoint(SketchLine* line)
{
    if(this->_p1 == line->_p1){
        return this->_p1;
    }else if(this->_p1 == line->_p2){
        return this->_p1;
    }else if(this->_p2 == line->_p1){
        return this->_p2;
    }else if(this->_p2 == line->_p2){
        return this->_p2;
    }else{
        printf("Not adjacent lines!!!\n");
        exit(1);
    }
}

bool SketchLine::containsDiffOrient(SketchLine* line)//返回两条线是不是有turning point
{
    if(this->chosen_orientation != line->chosen_orientation){
        return true;
    }else{
        if(this->chosen_orientation != intersectPoint(line)->chosen_orientation)
            return true;
        else
            return false;
    }
}

/*
 * 返回两条线的所有可能的关于坐标轴对称的方向,如果是空集说明不对称
 */
set<int> SketchLine::symmetryDirections(SketchLine *line){
    set<int> symmetry_direction;

    Vector3d p1_ref_x = this->_p1->getPointVec(); p1_ref_x.x() *= -1.0;
    Vector3d p2_ref_x = this->_p2->getPointVec(); p2_ref_x.x() *= -1.0;
    Vector3d p1_ref_y = this->_p1->getPointVec(); p1_ref_y.y() *= -1.0;
    Vector3d p2_ref_y = this->_p2->getPointVec(); p2_ref_y.y() *= -1.0;
    Vector3d p1_ref_z = this->_p1->getPointVec(); p1_ref_z.z() *= -1.0;
    Vector3d p2_ref_z = this->_p2->getPointVec(); p2_ref_z.z() *= -1.0;

    Vector3d l_p1 = line->_p1->getPointVec();
    Vector3d l_p2 = line->_p2->getPointVec();
    if(glob_vars::current_model.symmetry_x && isSameSegment(l_p1,l_p2,p1_ref_x,p2_ref_x)){
//    if(isSameSegment(l_p1,l_p2,p1_ref_x,p2_ref_x)){
        symmetry_direction.insert(Constants::X_AXIS);
    }
    if(glob_vars::current_model.symmetry_y && isSameSegment(l_p1,l_p2,p1_ref_y,p2_ref_y)){
//    if(isSameSegment(l_p1,l_p2,p1_ref_y,p2_ref_y)){
        symmetry_direction.insert(Constants::Y_AXIS);
    }
    if(glob_vars::current_model.symmetry_z && isSameSegment(l_p1,l_p2,p1_ref_z,p2_ref_z)){
//    if(isSameSegment(l_p1,l_p2,p1_ref_z,p2_ref_z)){
        symmetry_direction.insert(Constants::Z_AXIS);
    }
    return symmetry_direction;
}

bool SketchLine::isAxisSymmetry(SketchLine* line, int symmPlane){
    assert(this != line);
    assert(symmPlane == Constants::PLANE_XY ||
           symmPlane == Constants::PLANE_XZ ||
           symmPlane == Constants::PLANE_YZ);

    Vector3d p1_ref = this->_p1->getPointVec();
    Vector3d p2_ref = this->_p2->getPointVec();
    switch (symmPlane){
        case Constants::PLANE_XY:
            p1_ref.z() *= -1.0;
            p2_ref.z() *= -1.0;
            break;

         case Constants::PLANE_YZ:
            p1_ref.x() *= -1.0;
            p2_ref.x() *= -1.0;
            break;

        case Constants::PLANE_XZ:
            p1_ref.y() *= -1.0;
            p2_ref.y() *= -1.0;
            break;
        default:
            printf("Error! No indicated symmetry %d\n", symmPlane);
    }

    Vector3d l_p1 = line->_p1->getPointVec();
    Vector3d l_p2 = line->_p2->getPointVec();
    if(isSameSegment(l_p1,l_p2,p1_ref,p2_ref)){
        return true;
    }else {
        return false;
    }
}

void DFS_symmetry_lines(SketchLine* l, set<SketchLine*>& result){
    result.insert(l);
    set<SketchLine*> adj_edges;
    if(l->ref_symm_line_x != NULL) adj_edges.insert(l->ref_symm_line_x);
    if(l->ref_symm_line_y != NULL) adj_edges.insert(l->ref_symm_line_y);
    if(l->ref_symm_line_z != NULL) adj_edges.insert(l->ref_symm_line_z);
    for(auto& e : adj_edges){
        if(result.find(e) == result.end())//1.需要在这个component上 2)rigid 3)当前数据集中还没有
            DFS_symmetry_lines(e,result);
    }
}

set<SketchLine*> SketchLine::getAllSymmetryLines()//得到这条线相关联的所有symmetry lines
{
    set<SketchLine*> result;
    DFS_symmetry_lines(this,result);
    return result;
}

bool SketchLine::containsSamePossiOrient(SketchLine* line)
{
    vector<int> possi_orientation_set;
    if(line->intLength() > 2){
        possi_orientation_set = line->possi_orient_set;
    }else{
        int line_tag = line->possi_orientation;
        if(line_tag==Constants::PLANE_XZ || line_tag==Constants::PLANE_XY || line_tag==Constants::PLANE_YZ){
            possi_orientation_set.push_back(line_tag);
        }else if(line_tag == Constants::PLANE_YZ_OR_XZ){
            possi_orientation_set.push_back(Constants::PLANE_YZ);
            possi_orientation_set.push_back(Constants::PLANE_XZ);
        }else if(line_tag == Constants::PLANE_XY_OR_YZ){
            possi_orientation_set.push_back(Constants::PLANE_XY);
            possi_orientation_set.push_back(Constants::PLANE_YZ);
        }else if(line_tag == Constants::PLANE_XY_OR_XZ){
            possi_orientation_set.push_back(Constants::PLANE_XY);
            possi_orientation_set.push_back(Constants::PLANE_XZ);
        }else{
            printf("Layer classification error!!!!!\n");
            cin.get();
        }
    }

    if(this->choosedUnnormalOrient())
        return false;

    for(auto& c : possi_orientation_set){
        if(this->chosen_orientation == c){
            return true;
        }
    }
    return false;
}

bool SketchLine::choosedUnnormalOrient()//返回此线是不是选择了一个不是possible_orientation的方向，对于tiny segment
{
    vector<int> possi_orientation_set;
    int line_tag = this->possi_orientation;
    if(line_tag==Constants::PLANE_XZ || line_tag==Constants::PLANE_XY || line_tag==Constants::PLANE_YZ){
        possi_orientation_set.push_back(line_tag);
    }else if(line_tag == Constants::PLANE_YZ_OR_XZ){
        possi_orientation_set.push_back(Constants::PLANE_YZ);
        possi_orientation_set.push_back(Constants::PLANE_XZ);
    }else if(line_tag == Constants::PLANE_XY_OR_YZ){
        possi_orientation_set.push_back(Constants::PLANE_XY);
        possi_orientation_set.push_back(Constants::PLANE_YZ);
    }else if(line_tag == Constants::PLANE_XY_OR_XZ){
        possi_orientation_set.push_back(Constants::PLANE_XY);
        possi_orientation_set.push_back(Constants::PLANE_XZ);
    }else{
        printf("Layer classification error!!!!!\n");
        cin.get();
    }
    for(auto& c : possi_orientation_set){
        if(this->chosen_orientation == c)
            return false;
    }
    assert(this->intLength()<=2);
    return true;
}

int SketchLine::isConnected(SketchLine* line){
    if(line == this){ //如果是自己，默认不连接
        return false;
    }
    if(_p1 == line->_p1)
        return 1;
    else if(_p1==line->_p2)
        return 2;
    else if(_p2 == line->_p1)
        return 3;
    else if(_p2==line->_p2)
        return 4;
    else
        return 0;
}

bool SketchLine::integer_length_check(SketchPoint* p1, SketchPoint * p2) {
    double length = (p1->getPointVec() - p2->getPointVec()).norm();
    double int_length = round(length);
    if(fabs(length-int_length)>0.05){
        return false;
    }
    return true;
}

bool SketchLine::contains(Vector3d point)//判断一条线包含一个点
{
    double step = (double)1/this->length();
    for(double t = -0.0001;t<=1.0001;t+=step){
        Vector3d p = (1-t) * _p1->getPointVec() + t * _p2->getPointVec();
        if((point-p).norm()<0.05){
            return true;
        }
    }
    return false;
}

bool SketchLine::isOrthorgnalLine() const
{
    Vector3d direction = this->_p1->getPointVec() - this->_p2->getPointVec();
    direction = direction/direction.norm();
    if(fabs(fabs(direction.dot(Vector3d(1,0,0)))-1)<0.05 || fabs(fabs(direction.dot(Vector3d(0,1,0)))-1)<0.05 || fabs(fabs(direction.dot(Vector3d(0,0,1)))-1)<0.05){
        return true;
    }
    return false;
}

vector<LayerHole*> SketchLine::getInternalHoles() const {
    assert(!this->unit_edges.empty());
    set<LayerHole*> result_holes;
    for(auto& ue : this->unit_edges){
        if(ue->hole1->sketchPoint == NULL)  result_holes.insert(ue->hole1);
        if(ue->hole2->sketchPoint == NULL)  result_holes.insert(ue->hole2);
    }
    assert(this->chosen_orientation != 0);
    for(auto& lh : result_holes){
        lh->orientation = this->chosen_orientation;
    }
    vector<LayerHole*> holes_vec(result_holes.begin(), result_holes.end());
    return holes_vec;
}

int SketchLine::randomChangeOrientation()
{
    assert(!this->possi_orient_set.empty());
    if(this->possi_orient_set.size() == 1){
        //doing nothing..
    }else{
        assert(this->possi_orient_set.size() <= 3);
        random_shuffle(possi_orient_set.begin(),possi_orient_set.end());
        for(auto& c : this->possi_orient_set){
            if(this->chosen_orientation != c){
                this->chosen_orientation = c;
                return c;
            }
        }

    }
}