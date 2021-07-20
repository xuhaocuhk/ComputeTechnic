//
// Created by student on 4/26/2019.
//
#include <configure/Constants.h>
#include <configure/global_variables.h>
#include "Symmetry.h"

vector<BeamTemplate *>
Symmetry::get_all_reflect_beams_general(const vector<BeamTemplate *> &beams, int ref_direction, int layer_flag) {
    vector<BeamTemplate *> result;
    Matrix3d ref_mat = Matrix3d::Identity();

    switch (ref_direction) {
        case Constants::X_AXIS: {
            ref_mat(0, 0) *= -1;
            break;
        }
        case Constants::Y_AXIS: {
            ref_mat(1, 1) *= -1;
            break;
        }
        case Constants::Z_AXIS: {
            ref_mat(2, 2) *= -1;
            break;
        }
        default: {
            printf("Symmetry direction error!!!\n");
            cin.get();
        }
    }

    for (const auto &b : beams) {
        BeamTemplate *nb = new BeamTemplate(*b);
        if ((layer_flag == Constants::PLANE_XY && ref_direction == Constants::Z_AXIS) ||
            (layer_flag == Constants::PLANE_XZ && ref_direction == Constants::Y_AXIS) ||
            (layer_flag == Constants::PLANE_YZ && ref_direction == Constants::X_AXIS))
            nb->layer = -nb->layer;
        switch (ref_direction) {
            case Constants::X_AXIS: {
                nb->translation.x() *= -1.0;
                nb->transform(ref_mat);
                break;
            }
            case Constants::Y_AXIS: {
                nb->translation.y() *= -1.0;
                nb->transform(ref_mat);
                break;
            }
            case Constants::Z_AXIS: {
                nb->translation.z() *= -1.0;
                nb->transform(ref_mat);
                break;
            }
            default: {
                printf("Symmetry direction error!!!\n");
                cin.get();
            }
        }
        result.push_back(nb);
    }
    return result;
}

vector<Connector *> Symmetry::get_all_reflect_conns_general(const vector<Connector *> &m_conn, int ref_direction) {

    vector<Connector *> result;
    Matrix3d ref_mat = Matrix3d::Identity();
    switch (ref_direction) {
        case Constants::X_AXIS: {
            ref_mat(0, 0) *= -1;
            break;
        }
        case Constants::Y_AXIS: {
            ref_mat(1, 1) *= -1;
            break;
        }
        case Constants::Z_AXIS: {
            ref_mat(2, 2) *= -1;
            break;
        }
        default: {
            printf("Symmetry direction error!!!\n");
            cin.get();
        }
    }
    for (const auto &c : m_conn) {
        Connector *nc = new Connector(*c);

        switch (ref_direction) {
            case Constants::X_AXIS: {
                nc->translation.x() *= -1.0;
                nc->transform(ref_mat);
                break;
            }
            case Constants::Y_AXIS: {
                nc->translation.y() *= -1.0;
                nc->transform(ref_mat);
                break;
            }
            case Constants::Z_AXIS: {
                nc->translation.z() *= -1.0;
                nc->transform(ref_mat);
                break;
            }
            default: {
                printf("Symmetry direction error!!!\n");
                cin.get();
            }
        }
        result.push_back(nc);
    }
    return result;
}

vector<BeamHole *>
Symmetry::get_all_reflect_beamholes_general(const vector<BeamHole *> &m_beamholes, int ref_direction, int layer_flag) {
    vector<BeamHole *> result;
    Matrix3d ref_mat = Matrix3d::Identity();
    switch (ref_direction) {
        case Constants::X_AXIS: {
            ref_mat(0, 0) *= -1;
            break;
        }
        case Constants::Y_AXIS: {
            ref_mat(1, 1) *= -1;
            break;
        }
        case Constants::Z_AXIS: {
            ref_mat(2, 2) *= -1;
            break;
        }
        default: {
            printf("Symmetry direction error!!!\n");
            cin.get();
        }
    }

    for (const auto &bh : m_beamholes) {

        switch (ref_direction) {
            case Constants::X_AXIS: {
                bh->pos.x() *= -1.0;
                bh->normal = ref_mat * bh->normal;
                break;
            }
            case Constants::Y_AXIS: {
                bh->pos.y() *= -1.0;
                bh->normal = ref_mat * bh->normal;
                break;
            }
            case Constants::Z_AXIS: {
                bh->pos.z() *= -1.0;
                bh->normal = ref_mat * bh->normal;
                break;
            }
            default: {
                printf("Symmetry direction error!!!\n");
                cin.get();
            }
        }
        result.push_back(bh);
    }
    return result;
}


vector<BeamTemplate *>
Symmetry::general_reflct(int layertag, double current_layer_coor, double ref_layer_coor, vector<BeamTemplate *> beams) {
    double center_coor = (current_layer_coor + ref_layer_coor) / 2;

    vector<BeamTemplate *> result;
    for (auto &b : beams) {
        BeamTemplate *nb = new BeamTemplate(*b);
        Vector3d coordinate = b->getCurrentComponentHoles().back();
        switch (layertag) {
            case Constants::PLANE_XZ ://xz plane
                nb->translate(Vector3d(0, 2 * (center_coor - coordinate.y()), 0));
                break;
            case Constants::PLANE_XY ://xy plane
                nb->translate(Vector3d(0, 0, 2 * (center_coor - coordinate.z())));
                break;
            case Constants::PLANE_YZ://yz plane
                nb->translate(Vector3d(2 * (center_coor - coordinate.x()), 0, 0));
                break;
        }
        result.push_back(nb);
        if (glob_vars::print_flag) {
            nb->toLDrawFormat(5);
            b->toLDrawFormat(4);
        }
    }
    return result;
}

vector<BeamTemplate *> Symmetry::getSymmetryComponentsAsBeams(const vector<BeamTemplate *> &c_beams,
                                                              int plane_flag,
                                                              const set<tuple<SketchGroup *, vector<int>>> &symm_groups)//与上面方法不同的是这个函数是得到所有的beam group的beams，而不是一个
{
    assert(!symm_groups.empty());
    vector<BeamTemplate *> result;
    for (auto &group : symm_groups) {
        vector<BeamTemplate *> result_group(c_beams.begin(), c_beams.end());
        vector<int> symm_directions = std::get<1>(group);
        for (int direction : symm_directions) {
            result_group = Symmetry::get_all_reflect_beams_general(result_group, direction, plane_flag);
        }
        result.insert(result.end(), result_group.begin(), result_group.end());
    }
    return result;
}
