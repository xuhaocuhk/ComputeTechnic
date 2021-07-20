//
// Created by 徐豪 on 2018/4/23.
//

#ifndef LEGOGENERATION_MAIN_SKETCHGROUP_H
#define LEGOGENERATION_MAIN_SKETCHGROUP_H


#include "SketchLine.h"
#include <vector>
#include <tuple>

using std::vector;
using std::tuple;

class SketchGroup {

public:
    bool connect(SketchGroup* );//return whether this group is connect with the given group
    int getOrientationTag();
    void writeSketchToObj(vector<SketchPoint*> points, std::string remark);
    int totalSketchLength();
    set<int> symmetryDirections(SketchGroup *);//return whether this two groups are symmetry
    bool isConnectorComponent();
    std::tuple<int,int,double,double> getPossibleOriensAndCoord();//(0,2) (1,3)分别是两个方向以及所对应的坐标
    double getLayerCoordinate();
    void assignOrientInfo();
    bool isSelfSymmetry();//返回这个group是不是自己关于一个平面对称, TODO:目前只检查y平面

    vector<SketchGroup*> getAllSymmetryGroups();//得到当前group的所有symmetry groups，包括自己
    set<tuple<SketchGroup*,vector<int>>> getAllSymmetryGroupsWithDirecTag();//得到所有的对称group以及他们的方向，不包括自己！！, 如{X,Y}代表先沿x方向对称，再沿y方向对称
    

public:
    int orientation_tag;
    set<SketchLine*> lines;
    set<SketchGroup*> adjacentGroups;
    SketchGroup* symm_group_x = NULL;
    SketchGroup* symm_group_y = NULL;
    SketchGroup* symm_group_z = NULL;

};


#endif //LEGOGENERATION_MAIN_SKETCHGROUP_H
