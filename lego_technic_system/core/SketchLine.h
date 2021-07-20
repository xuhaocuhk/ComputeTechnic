//
// Created by 徐豪 on 2017/5/31.
// 用来表示输入的sketch的一条线段
//

#ifndef LEGOGENERATIONFROMSKETCH_ONEPLANE_SKETCH_LINE_H
#define LEGOGENERATIONFROMSKETCH_ONEPLANE_SKETCH_LINE_H

#include <cmath>
#include "SketchPoint.h"
#include "LayerHole.h"
#include "UnitEdge.h"
#include <Eigen/Dense>
#include <set>

using Eigen::Vector3d;
using std::vector;
using std::set;

class SketchPoint;
class LayerHole;
class UnitEdge;

class SketchLine {

public:
    SketchLine();
    SketchLine(SketchPoint *p1, SketchPoint *p2,bool is_feature_line);
    Eigen::Vector3d getVector();
    set<SketchLine*> getAdjLines();//return all adacent lines
    set<SketchLine*> getAllAdjLines(set<SketchLine*>);//返回所有邻接的非major plane的lines,包括自己
    set<SketchLine*> getAllAdjLinesByPossiOrient(set<SketchLine *>, int line_tag);
    set<SketchLine*> getAllAdjLinesByChosenOrient(set<SketchLine *>, int line_tag);
    bool containsDiffOrient(SketchLine*);//返回两条线是不是有turning point
    SketchPoint* intersectPoint(SketchLine*);//return the intersection point of the given two lines

    set<int> symmetryDirections(SketchLine *);
    bool isAxisSymmetry(SketchLine* line, int symmPlane);
    set<SketchLine*> getAllSymmetryLines();//得到这条线相关联的所有symmetry lines,包括自己！！！
    bool containsSamePossiOrient(SketchLine*);//返回传入的line的possible orientation是不是含有现在的line的chosen orientation
    bool choosedUnnormalOrient();//返回此线是不是选择了一个不是possible_orientation的方向，对于tiny segments
    int isConnected(SketchLine* line);
    bool integer_length_check(SketchPoint * p1, SketchPoint * p2);
    bool contains(Vector3d);//判断一条线包含一个点,assume这个点是整数单位点
    bool isOrthorgnalLine() const;
    Vector3d center();
    double length();
    int intLength();

    //得到这条线上的hole，方向是根据chosen orientation,不包括它的端点！！！！！！！！
    vector<LayerHole*> getInternalHoles() const;

    //随机从现有的可能的valid orientation中选取一个
    int randomChangeOrientation();

public:
    int _id;
    static int max_id;
    SketchPoint *_p1;
    SketchPoint *_p2;
    int possi_orientation = 0;//使用二进制表示线段所在的plane，6代表xy plane，5代表xz plane, 3代表yz plane
    vector<int> possi_orient_set;
    int chosen_orientation = 0;
    bool feature_line = false;
    SketchLine* ref_symm_line_x = NULL;
    SketchLine* ref_symm_line_y = NULL;
    SketchLine* ref_symm_line_z = NULL;
    vector<UnitEdge*> unit_edges;
    set<SketchLine*> symm_group;//自己的symmetry line永远不能包括自己！！！
};


#endif //LEGOGENERATIONFROMSKETCH_ONEPLANE_SKETCH_LINE_H
