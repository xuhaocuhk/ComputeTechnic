//
// Created by 徐豪 on 2017/10/13.
//

#ifndef LEGOGENERATIONFROMSKETCH_ONEPLANE_GRIDSYSTEM_H
#define LEGOGENERATIONFROMSKETCH_ONEPLANE_GRIDSYSTEM_H

#include <Eigen/Dense>
#include <queue>
#include "SketchLine.h"
#include "LayerHole.h"

#include "util/util.h"
#include "LayerGraph.h"
#include "Model.h"
#include "ConstructionGraph.h"

using Eigen::Vector3d;
using Eigen::Vector3i;



class LEGOGenerator {

public:
    /******* Initialization ******/
    LEGOGenerator();
    void chooseModel();//choose which model we want to generate

    /******* Sketch Decomposition ******/
    void extractComponents();
    void computeBeamPlacements();

    /******* Generating candidates *******/
    void stage_two_generation();

public:
    ConstructionGraph* major_compont_graph;

};



#endif //LEGOGENERATIONFROMSKETCH_ONEPLANE_GRIDSYSTEM_H
