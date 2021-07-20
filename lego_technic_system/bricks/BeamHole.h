//
// Created by 徐豪 on 2017/11/13.
//

#ifndef LEGOGENERATION_MAIN2_BEAMHOLE_H
#define LEGOGENERATION_MAIN2_BEAMHOLE_H

#include "BeamTemplate.h"
#include "Eigen/Dense"
#include "connector/Connector.h"
#include <set>


using Eigen::Vector3d;
using std::set;
using std::vector;

class BeamTemplate;
class Connector;
class BeamHoleGraph;
class BeamInstance;
class LayerHole;
//每一个对象都代表 beam的一个点，它有可能是circle的也可能是axle的


class BeamHole {

public:
    BeamHole(Vector3d pos, bool isAxle, Vector3d normal, int layer_num);
    set<BeamHole*> getPossiConnectedBeamHoles();//得到所有**一次直接**通过possible_connectors连接的所有的BeamHoles
    set<BeamHole*> getAllPotentialConnectedBeamHoles();//得到所有间接通过possible_connectors连接的所有的BeamHoles
    set<BeamHole*> getAllPotentialConnectedBeamHoles(int number);//得到所有间接通过possible_connectors连接的所有的BeamHoles,并控制返回数量为number
    bool alreadyBeenInserted();
    //这个方法执行的前提是两个beam之间本身没有connector
    void toLDrawFormat(int color = 5);
    string toLDrawFormatAsString(int color = 5);

    int layer_number_on_beam = 999412; // the layer number on its corresponding beam

    Vector3d pos;
    bool isAxle;
    Vector3d normal;
    BeamTemplate *beam = NULL;//表示它属于的那个beam
    Connector* assigned_connector;

    std::vector< Connector* > possi_connectors; // all possible connectors represented as pointer

    set<BeamHole*> close_holes; // holes that are close the current hole

    ////The beamInstance this beamhole belongs to
    BeamInstance* beamInstance = NULL;
    ////The layerhole this beamhole belongs to
    LayerHole* layerHole = NULL;

};


#endif //LEGOGENERATION_MAIN2_BEAMHOLE_H
