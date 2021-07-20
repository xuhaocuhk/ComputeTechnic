//
// Created by 徐豪 on 2017/11/3.
//

#ifndef LEGOGENERATION_MAIN2_GRADER_H
#define LEGOGENERATION_MAIN2_GRADER_H


#include "core/LayerHole.h"
#include "core/SketchGraph.h"
#include "core/BeamLayout.h"
#include "core/ConstructionGraph.h"
#include "GradeRecorder.h"
#include <vector>

class ConstructionGraph;

class Grader {

public:
    /* 2D grader */
    static double deviation2sketch( BeamLayout&);
    static double deviation2sketch_online( BeamLayout&, GradeRecorder& );//locally update version, update gradeRecorder outside
    static double deviation2sketch_3D(ConstructionGraph&);
    static double deviation2sketch_online_3D(ConstructionGraph&);


    static double simplicity(BeamLayout&);
    static double simplicity_online(BeamLayout&, GradeRecorder&);
    static double simplicity_3D(ConstructionGraph&);
    static double simplicity_online_3D(ConstructionGraph&);

    static double rigidity_transitive(BeamLayout &);
    static double rigidity_transitive_online(BeamLayout&, GradeRecorder&);
    static double rigidity_transitive_3D(ConstructionGraph&);
    static double rigidity_transitive_online_3D(ConstructionGraph&);

    static double gaps(BeamLayout&);
    static double gaps_online(BeamLayout&, GradeRecorder&);
    static double gaps_3D(ConstructionGraph&);
    static double gaps_online_3D(ConstructionGraph&);

    static double layer_bound(BeamLayout&);
    static double layer_bound_online(BeamLayout&, GradeRecorder&);
    static double layer_bound_3D(ConstructionGraph&);
    static double layer_bound_online_3D(ConstructionGraph&);

    static double layer_symmetry(BeamLayout&);//The symmetry beam pair should be ideally have same layer number, this function evaluate this
    static double layer_symmetry_online(BeamLayout&, GradeRecorder&);
    static double layer_symmetry_3D(ConstructionGraph&);
    static double layer_symmetry_online_3D(ConstructionGraph&);

    static double completeness(BeamLayout&); //return the completeness of edge cover
    static double completeness_online(BeamLayout&, GradeRecorder&);
    static double completeness_3D(ConstructionGraph &);
    static double completeness_online_3D(ConstructionGraph &);
    static double completeness_online_3D_inferenced(ConstructionGraph& ); //put other components layer shifting to current component into consideration

    static double collision_3D(ConstructionGraph &); //return the normalized collision ratio
    static double collision_num_3D(ConstructionGraph &); // return the number of collison holes
    //TODO: implement a local update version here

    /**** Grader for connction ****/
    static double connection_score_3D(ConstructionGraph &);


    static double Nonpin_head_ratio(const vector<Connector *> &connectors);


    static double overall_score_2D(BeamLayout &, bool print_flag, string &str);
    static double overall_score_2D_online(BeamLayout &, GradeRecorder &, bool print_flag, string &str);
    static double overall_score_3D(ConstructionGraph &, bool consider_connection = false ,bool consider_collision = false, bool print_flag = false);



    /* old 3D grader */
    static double pinhead_ratio(const vector<Connector  > &);
    static int pin_num(const vector<Connector*>&);

    static double sketchOrthorgNum_enhenceNonOrthorg(const SketchGraph*);
};


#endif //LEGOGENERATION_MAIN2_GRADER_H
