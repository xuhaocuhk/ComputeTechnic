//
// Created by 徐豪 on 2017/10/14.
//

#ifndef LEGOGENERATION_MAIN_HOLE_H
#define LEGOGENERATION_MAIN_HOLE_H

#include "bricks/BeamTemplate.h"
#include "UnitEdge.h"
#include <set>
#include <vector>

using std::set;
using Eigen::Vector3d;
using std::vector;

class BeamTemplate;
class BeamHole;
class SketchPoint;
class UnitEdge;
class BeamLayout;
class BeamInstance;


//Beam的Hole
class LayerHole{

//To compare two layerhole according to location
public:
    struct LayerHoleComp {
        bool operator() (LayerHole* h1, LayerHole* h2) const
        {
            return h1->_id > h2->_id;
        }
    };


public:
    LayerHole(double x, double y, double z, int orient);
    Vector3d position() const { return Vector3d(_x,_y,_z); }
    void add_adj_hole(LayerHole*);

    void addBeam(BeamTemplate *);
    bool fully_covered() const;//当一个hole与它的所有adjacent hole都有相同的beam的时候，这时这个hole被fully covered
    int cover_status();//返回adjacent holes 中被cover的数量
    bool is_boundary_corner();
    bool is_T_joint();
    void print();
    void toLdrawFormat(int color = 11);
    std::string toLdrawFormatAsString(int color = 11);

    set<LayerHole*> getAllSymmetryHoles();
    set< std::tuple<LayerHole*, vector<int> > > getAllSymmetryLayerHolesWithDirecTag();
    vector<BeamHole*> getBeamHolesOnThisLayerHole();
    vector<BeamHole*> getBeamHolesOnthisLayerHoleGivenBeam(BeamInstance*);

    int _id;
    static int max_id;
    double _x,_y,_z;
    int orientation;
    set<BeamTemplate *> added_beams;
    set<LayerHole*, LayerHoleComp> adj_holes;
    set<LayerHole*, LayerHoleComp> close_holes;//不是在一条线上的adjacent holes,但距离小于一，由于线之间的角度太小导致。
    vector<BeamTemplate *> possible_beams;
    SketchPoint* sketchPoint = NULL;//如果当前layerhole在一个sketchpoint上面则赋值，如果不在则为空
    bool featured = false;
    bool to_delete = false;
    bool symmetry_part = false;
    set<int> fixed_layer;//通过joint判断得出的一定要有beam的layer
    set<UnitEdge*> unit_edges;//此hole相连的unit edges

    LayerHole* symmtry_hole = NULL; //Symmetry hole inside a component

    LayerHole* symmetry_hole_x = NULL;
    LayerHole* symmetry_hole_y = NULL;
    LayerHole* symmetry_hole_z = NULL;

    BeamLayout * belonged_beamLayout = NULL; // The BeamLayout this hole belongs to

    static BeamTemplate getUnitHole(Vector3d pos, int orien_tag);
};



#endif //LEGOGENERATION_MAIN_HOLE_H
