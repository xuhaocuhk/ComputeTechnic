//
// Created by 徐豪 on 2017/6/3.
//

#ifndef LEGOGENERATIONFROMSKETCH_ONEPLANE_BEAM_H
#define LEGOGENERATIONFROMSKETCH_ONEPLANE_BEAM_H

#include <iostream>
#include "Eigen/Dense"
#include <string>
#include <set>
#include <core/UnitEdge.h>
#include "../core/SketchPoint.h"
#include "../core/LayerHole.h"
#include "BeamHole.h"

using Eigen::Vector3d;
using Eigen::Vector3i;
using std::vector;
using std::string;
using Eigen::Matrix3d;

class Connector;

class SketchPoint;

class LayerHole;

class LayerGraph;

class BeamTemplate {

public:
    BeamTemplate(string name, vector<Vector3d> component_holes, string LD_id, Vector3d normal, bool is_straight,
                 int color = 15);

    BeamTemplate(const BeamTemplate &b);//复制构造函数

    BeamTemplate &transform(const Matrix3d &mat);

    BeamTemplate &translate(const Vector3d &trans);


    vector<Vector3d> getCurrentComponentHoles() const;

    int holeNum() const;

    Vector3d getNormal() const;

    void resetTransformation();

    void print() const;

    void toLDrawFormat() const;

    void toLDrawFormat(int color) const;

    string toLDrawFormatAsString(int color = 15) const;

    string toIndividualLDrawAsString(int color = 15) const;

    bool operator==(const BeamTemplate &);

    //取两个集合的交集，结果保存在v1中
    std::set<int> getFeatureHoleID() const;//得到一个beam上面的feature point(两端的point) cover的 hole的id
    std::set<LayerHole *> getFeatureHole() const;//得到一个beam上面的feature hole的指针引用
    bool equal_on_cover_id(const BeamTemplate &, vector<LayerHole *> hole_sequence = vector<LayerHole *>());

    double distance(LayerHole *);//返回这个beam与给定hole的最近距离
    bool isSymmetry(BeamTemplate &b);//返回两个beam是不是symmetry的


public:
    int _id = -1;
    static int max_id;
    int color;
    string name;  //Beam的分类名称
    string ldraw_id;   //在ldraw的.dat的名称
    int layer = 0;
    bool is_straight = false;
    double local_score;
    vector<LayerHole *> covered_holes;//此beam cover的hole的指针
    vector<int> axle_index;

    bool feature_beam;

    BeamTemplate *refl_symmetry_beam = NULL;

    vector<BeamTemplate> sub_beams;

    vector<UnitEdge *> covered_edges;

public:
    Matrix3d transf_mat;
    Vector3d translation;
    Vector3d normal; // The orientation of the hole. Very important!!
    vector<Vector3d> component_holes;   //beam的基本属性，存储有多个hole在object坐标系下的坐标
};

#endif //LEGOGENERATIONFROMSKETCH_ONEPLANE_BEAM_H
