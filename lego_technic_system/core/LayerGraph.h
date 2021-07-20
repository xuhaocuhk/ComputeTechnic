//
// Created by 徐豪 on 2017/11/3.
//

#ifndef LEGOGENERATION_MAIN2_LAYERGRAPH_H
#define LEGOGENERATION_MAIN2_LAYERGRAPH_H


#include "LayerHole.h"
#include "SketchGroup.h"
#include <map>
#include "configure/Constants.h"

class LayerGraph {

public:

    /******** Basic Operation *********/
    LayerGraph(){};
    LayerGraph(SketchGroup*);
    LayerHole* getHole(int index);
    int hole_num() const;
    int beam_num() const;
    void establishAdjacentRelation(vector<SketchLine*>&);

    /******** Status Checking *********/
    bool isConnectorComponent();
    bool isUniqueOrient(); //whether the layergraph dont have a determined orientation yet
    int getCurrentPlaneFlag();
    Eigen::Vector3d getCurrentPlaneNormal();
    bool containSymmetryComponentsAllDirection();

    double distance(LayerGraph*);// return the shortest distance between two graphs
    //使用比较general的方法循环遍历所有的距离为一的hole pairs
    int contactPointsNum(const LayerGraph&); //return the contacting number(number of connectors connect them) between two components.

    /****** General Interface Function ********/
    void pre_compute_single_holes(int plane_flag); //add single-hole solution to the possible beams
    void pre_compute_all_placement(int plane_flag,
                                   const vector<BeamTemplate *> &gend_beams);//计算所有的可能的beam，将结果保存在hole中,没有layer

    /******************************** Strategies ****************************************/

    /******* Debug Information *********/
    string getCurrentHolesAsString(int color);

    /**** Interface Function (Symmetry Method) ****/
    //hole之间的symmetry包括自己和自己的symmetry，beam之间的symmetry包括自己，unit edge之间的symmetry不包括自己和自己的symmetry
    bool establishSymmetryRelation();//此方法计算symmetry的关系并存储
    void erasePlaneSymmetryRelation();
    void establishSymmetryUnitEdgeRelation();//基于layer hole的symmetry关系，计算unit edge的symmetry关系并存储

    ////For minor component only : update beam orientation accorfing to layergraph orientation
    void updateBeamOrientation();


public:
    int orientation_index = 0;//表示现在这个对象已经调用beam search方法多少遍了
    vector<BeamTemplate *> all_possi_beams;
    vector<LayerHole*> holes;
    vector<UnitEdge*> unit_edges;
    vector<BeamTemplate *> beams; // 当前已选择的beam
    Vector3d normal;
    int orientation_tag = -1;
    int component_type = 0;

    SketchGroup* sketchGroup = NULL;

private:
    void perm_layer(int num_slots, int max_layer, int* result, int beam_size);


};


#endif //LEGOGENERATION_MAIN2_LAYERGRAPH_H
