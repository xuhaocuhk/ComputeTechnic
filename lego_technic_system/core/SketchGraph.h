//
// Created by 徐豪 on 2017/7/4.
//

#ifndef LEGOGENERATIONFROMSKETCH_ONEPLANE_SKETCHGRAPH_H
#define LEGOGENERATIONFROMSKETCH_ONEPLANE_SKETCHGRAPH_H

#include <vector>
#include <string>
#include "SketchLine.h"
#include "SketchGroup.h"
#include <Eigen/Dense>
#include <map>

class SketchGraph {

public:
    SketchGraph(std::vector<SketchLine*>* lines, std::vector<SketchPoint*>* points):
            m_lines(lines),m_points(points){}
    inline std::vector<SketchLine*>* getLines(){return m_lines; }
    inline std::vector<SketchPoint*>* getPoints() const{return m_points;}
    void initOrientationCandidates() const;
    void sketchInitFreeTinyLines() const;
    void estabilishAdjacentGroup();
    void writeObj(std::string path);

    void estabilshSketchSymmetry();//Establish symmetry on sketch line level

    /******** Decomposition *******/
    void initLinesOrientation();//使用greedy的sweep plane来对每一条线段赋予方向
    void randomInitLinesOrientation();
    void initPointsOrientation();
    double doRandomOperation_AllSymmetry(double& curr_score, SketchLine*& changed_line, int& prev_chosen);
    void undoOperationAllSymmetry(SketchLine *changed_line, int &prev_chosen);
    void estimateLocalBeamOrientation();
    void decomposeBySA();
    void groupLinesByChosenDirection(set<SketchLine *> &lines, vector<SketchGroup *> &result_groups);
    int totalSketchLength();

public:
    static void establishSymmetryGroupRelation(vector<SketchGroup *> &groups);
    static void eraseSymmetryGroups(vector<SketchGroup*>& all_groups);

public:

    std::vector<SketchGroup*> group_xz;
    std::vector<SketchGroup*> group_yz;
    std::vector<SketchGroup*> group_xy;
    std::vector<SketchGroup*> uncertain_group;
    vector<SketchGroup*> all_groups;

public:
    std::vector<SketchLine*>* m_lines;
    std::vector<SketchPoint*>* m_points;


    SketchLine* getGeneralizedDesiredLine(const set<SketchLine *>&, int plane_tag);
    SketchLine* getExactlyDesiredLine(const set<SketchLine*>&, int plane_flag);

    void sweepPlane(int plane_flag, set<SketchLine *> &lines,
                    vector<SketchGroup*> &result_group, bool erase); //将lines按照相应平面decompose结果放在group中，并且根据erase变量来看是否需要erase已经group 的line

    void sweepPlaneByChosenOrient(int orientation_tag, set<SketchLine *> &lines,
                                  vector<SketchGroup*> &result_group, bool erase); //将lines按照 decomposition 算法之后的chosen 平面来decompose结果放在group中，并且根据erase变量来看是否需要erase已经group 的line

    void findKLargestComponents(set<SketchLine *> &lines, int k, vector<SketchGroup *> &k_groups);

    void findPossibleDecompose(set<SketchLine *> &lines, int k, vector<SketchGroup *> &group_sequence,
                               vector<vector<SketchGroup *>> &results);

};


#endif //LEGOGENERATIONFROMSKETCH_ONEPLANE_SKETCHGRAPH_H
