//
// Created by 徐豪 on 2018/4/18.
//

#ifndef LEGOGENERATION_PLANE_GENERATION2_GLOBAL_VARIABLES_H
#define LEGOGENERATION_PLANE_GENERATION2_GLOBAL_VARIABLES_H

#include "core/Model.h"
#include "hard_code/SymmetryRelation.h"
#include "core/SketchGraph.h"
#include "core/GuidingGraph.h"
#include "bricks/BrickFactory.h"

/********* MACROS For Control *********/
//#define OUTPUT_ANIMATION
//#define OUTPUT_ANIMATION
//#define CROSSBOW_HACKING
//#define HACKING

/********* Global variables *********/
namespace glob_vars{
    /********* Debugging *********/
    extern string current_debug_dir;
    const bool print_flag = false;
    extern std::ofstream config_file;

    /********* Input and Output *********/
    const string PRE_FIX = "../LEGO_Technic_data/";
    extern int current_input;
    extern model current_model;
    extern string sketch_path;


    /********* Data Structures *********/
    extern SketchGraph* sketchGraph;
    extern GuidingGraph* guidingGraph;
    extern BrickFactory *brickFactory;

    /*** generation configuration ***/
    const double adj_comp_dist = 2.3;

}

namespace config{
    extern vector<model> model_names;
}

#endif //LEGOGENERATION_PLANE_GENERATION2_GLOBAL_VARIABLES_H
