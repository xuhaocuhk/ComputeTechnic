//
// Created by 徐豪 on 2017/7/4.
//
#include "util/util.h"
#include "../core/LayerGraph.h"
#include "../core/SketchPoint.h"

int SketchPoint::max_id = 0;
using Eigen::Vector3d;

SketchPoint::SketchPoint(double x,double y,double z)
{
    _id = max_id++;
    this->_x = x; this->_y= y; this->_z=z;
}

SketchPoint::SketchPoint(const SketchPoint& p)
{
    this->_x = p._x;
    this->_y = p._y;
    this->_z = p._z;
}

void SketchPoint::addLine(SketchLine* line)
{
    lines.push_back(line);
}

Vector3d SketchPoint::getPointVec()
{
    return Vector3d(_x,_y,_z);
}

LayerHole* SketchPoint::getHole()
{
    this->layerhole->orientation = chosen_orientation;
    return this->layerhole;
}

void SketchPoint::chooseMinObjfuncDirection(){
//    int xy_count = 0, xz_count = 0, yz_count = 0;
//    auto adj_lines = this->getAdjLines();
//    for(auto& l : adj_lines){
//        switch(l->chosen_orientation){
//            case LayerGraph::PLANE_XY:{xy_count++; break; }
//            case LayerGraph::PLANE_YZ:{yz_count++; break; }
//            case LayerGraph::PLANE_XZ:{xz_count++; break; }
//            default:{ printf("Line orientation classification error!!!!\n"); }
//        }
//    }
//    this->chosen_orientation = xy_count>xz_count ? //选择三个方向中出现次数最多的那个方向
//                            (xy_count>yz_count? LayerGraph::PLANE_XY:LayerGraph::PLANE_YZ):
//                            (xz_count>yz_count? LayerGraph::PLANE_XZ:LayerGraph::PLANE_YZ);
    int min_obj_choice = 0;
    double min_score = 999.0;
    for(auto& c : {1,2,3}){
        this->chosen_orientation = c;
        double curr_score = scoreNonOrthorNum_enhenceNonOrthorg();
        if(curr_score<min_score){
            min_score = curr_score;
            min_obj_choice = c;
        }
    }
    this->chosen_orientation = min_obj_choice;
}

double SketchPoint::scoreNonOrthorNum(){
    double result = 0.0;
    auto adj_lines = this->getAdjLines();
    for(auto& l : adj_lines){
        if (l->chosen_orientation != this->chosen_orientation){
            result += 1.0;
        }
    }
    return result;
}

double SketchPoint::scoreNonOrthorNum_enhenceNonOrthorg(){//对于一个hole有多个neibor hole的情况，这个hole应该优先选择orthorgonal的hole，尽量不选择non-orthorgonal
    double result = 0.0;
    auto adj_lines = this->getAdjLines();
    for(auto& l : adj_lines){
        if (l->chosen_orientation != this->chosen_orientation){
            if(l->isOrthorgnalLine()){
                result +=1.0;
            }else{
                result += 1.6;
            }
        }
    }
    return result;
}