//
// Created by student on 4/26/2019.
//

#ifndef LEGO_TECHNIC_MAIN_SYMMETRY_H
#define LEGO_TECHNIC_MAIN_SYMMETRY_H

#include <bricks/BeamTemplate.h>

namespace Symmetry {

    vector<BeamTemplate *>
    general_reflct(int layertag, double current_layer_coor, double ref_layer_coor, vector<BeamTemplate *> beams);

    vector<BeamTemplate *>
    get_all_reflect_beams_general(const vector<BeamTemplate *> &, int ref_direction, int layer_flag);

    vector<Connector *> get_all_reflect_conns_general(const vector<Connector *> &, int ref_direction);

    vector<BeamHole *> get_all_reflect_beamholes_general(const vector<BeamHole *> &, int ref_direction, int layer_flag);

    vector<BeamTemplate *> getSymmetryComponentsAsBeams(const vector<BeamTemplate *> &,
                                                        int plane_flag,
                                                        const set<tuple<SketchGroup *, vector<int>>> &symm_groups);//与上面方法不同的是这个函数是得到所有的beam group的beams，而不是一个

}

#endif //LEGO_TECHNIC_MAIN_SYMMETRY_H
