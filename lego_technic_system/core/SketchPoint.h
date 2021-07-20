//
// Created by 徐豪 on 2017/5/31.
//

#ifndef LEGOGENERATIONFROMSKETCH_ONEPLANE_POINT_H
#define LEGOGENERATIONFROMSKETCH_ONEPLANE_POINT_H

#include <vector>
#include <Eigen/Dense>
#include "SketchLine.h"
#include "LayerHole.h"

using Eigen::Vector3d;

/**
 * 表示空间中的一个点，坐标的单位为LDU,1LDU = 4mm
 */
class Connector;
class SketchLine;
class LayerHole;

class SketchPoint {

public:
    SketchPoint(double x,double y,double z);
    SketchPoint(const SketchPoint& p);//用于复制的构造函数
    Vector3d getPointVec();
    void addLine(SketchLine* line);
    std::vector<SketchLine*> getAdjLines(){ return lines;}
    LayerHole* getHole();
    void chooseMinObjfuncDirection();//选择临边中出现次数最多的方向
    double scoreNonOrthorNum();
    double scoreNonOrthorNum_enhenceNonOrthorg();

public:
    int _id;
    double _x;
    double _y;
    double _z;
    LayerHole* layerhole = NULL;

    int chosen_orientation = 0;

    static int max_id;

    std::vector<SketchLine*> lines;

};


#endif //LEGOGENERATIONFROMSKETCH_ONEPLANE_POINT_H
