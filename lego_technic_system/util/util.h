//
// Created by 徐豪 on 2017/7/3.
//

#ifndef LEGOGENERATIONFROMSKETCH_ONEPLANE_UTIL_H
#define LEGOGENERATIONFROMSKETCH_ONEPLANE_UTIL_H
//random num range is min to max, template function
#define _USE_MATH_DEFINES

#include <cstring>
#include <iostream>
#include <vector>
#include "core/SketchGraph.h"
#include "core/Component.h"
#include <string>
#include "bricks/BeamTemplate.h"
#include <cmath>

using std::string;
using std::vector;

std::string getCurrentTimeAsString();
double cRandom(double min, double max);
int intRandom(int min, int max);//返回min 到 max之间的随机整数，包括min 与 max
double length(double p1x, double p1y, double p2x, double p2y);
double length(SketchPoint* p1, SketchPoint* p2);
std::string getBeamNo(double length,double precision);
bool beamExists(double length, double precision);
void SplitString(const std::string& s, std::vector<std::string>& v, const std::string& c);
SketchGraph* readSketchFromObjFile(string path);
Vector3d* line_closest(SketchLine *l1, SketchLine *l2); // get the closes two points on l1 and l2 resp., remember to delete the array!!
Eigen::MatrixXd getAffine_Mat(vector<SketchLine*> *lines);
Vector3d getOrienPlaneNormal(SketchGraph *sketchGraph);//get the main orientation plane's normal vector of the given data set
vector<vector<SketchLine*>> getMainOrieLines(SketchGraph *sketchGraph, Vector3d normal);
std::vector<int> range(int first, int end);
double distance_point_line(Vector3d point, Vector3d v_start, Vector3d v_end);

vector<BeamTemplate *> general_translate(Vector3d trans, vector<BeamTemplate *> &beams);

bool collide(vector<BeamTemplate *> &, vector<BeamTemplate *> &);//return if collision exists between two groups
bool collide(BeamTemplate &b, vector<BeamTemplate *> &);
bool isStraightLines(set<SketchLine*>);
bool isStraightNonOrthorgLine(set<SketchLine*>, int plane_tag);//这种情况下说明虽然是straight的但是plane是一定可以确定的
bool isSameSegment(const Vector3d &l1p1, const Vector3d &l1p2, const Vector3d &l2p1, const Vector3d &l2p2);
Vector3d getNormalFromOrientationTag(int layer_flag);
int min(int a, int b);
int max(int a, int b);

void perm_combination(int index, int set_number, const vector<vector<vector<BeamTemplate *>>> &sets, int *result,
                      vector<vector<BeamTemplate *>> &possible_groups);
bool layerGeneralizedEqual(int layer, int generalized_layer);
vector<UnitEdge*> getUnitEdgesFromHoles(vector<LayerHole*>&);//从layerhole 的集合中得到所有的小edges
void deleteElements(vector<BeamInstance*>&);
int pickWeightedRandElement(vector<int> weight);
bool contains(const vector<Connector>& conn_vec, const Connector& connector);
bool sortLayerByHoleNum(LayerGraph* lg1, LayerGraph* lg2);


#endif //LEGOGENERATIONFROMSKETCH_ONEPLANE_UTIL_H
