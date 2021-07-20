//
// Created by 徐豪 on 2017/11/3.
//
#define _USE_MATH_DEFINES

#include "bricks/BrickFactory.h"
#include <list>
#include <cfloat>
#include "LayerGraph.h"
#include "objectives/Grader.h"
#include <map>
#include <numeric>
#include "configure/global_variables.h"
#include <iomanip>
#include "util/util.h"
#include <fstream>
#include <util/Symmetry.h>

LayerGraph::LayerGraph(SketchGroup* group){

//    //For tiny component
//    if(group->lines.size() == 1 && (*group->lines.begin())->intLength()<=2){
//        auto line_holes = (*group->lines.begin())->getInternalHoles();
//        this->holes.insert(this->holes.end(),line_holes.begin(),line_holes.end());
//    } else
//    {
//        for(auto line : group->lines)
//        {
//            auto line_holes = line->getInternalHoles();
//            this->holes.insert(this->holes.end(),line_holes.begin(),line_holes.end());
//            if(group->isConnectorComponent() ||
//               layerGeneralizedEqual(line->_p1->chosen_orientation, group->layer_tag)){//remember to change normal
//                this->addHole(line->_p1->getHole());
//            }
//            if(group->isConnectorComponent() ||
//               layerGeneralizedEqual(line->_p2->chosen_orientation, group->layer_tag)){
//                this->addHole(line->_p2->getHole());
//            }
//        }
//    }
    assert( !group->lines.empty() );
    auto& first_line = *group->lines.begin();
    if( group->lines.size() == 1 && first_line->intLength() == 1 ){

            //do nothing, just ignore this group...

    }else if(group->lines.size() == 1 && first_line->intLength() == 2 ) {

        // line segment with length two
        auto interior_holes = first_line->getInternalHoles();
        this->holes.insert(this->holes.end(), interior_holes.begin(), interior_holes.end());

    }else{
        set<LayerHole*> end_points;
        for(auto& line : group->lines)
        {
            //add interior holes
            auto interior_holes = line->getInternalHoles();
            this->holes.insert(this->holes.end(), interior_holes.begin(),
                               interior_holes.end());
            //add end point holes to the set
            if(layerGeneralizedEqual( line->_p1->chosen_orientation, group->orientation_tag) )
                end_points.insert(line->_p1->getHole());
            if(layerGeneralizedEqual( line->_p2->chosen_orientation, group->orientation_tag) )
                end_points.insert(line->_p2->getHole());
        }

        //Add end holes
        this->holes.insert(this->holes.end(), end_points.begin(), end_points.end() );

    }

    set<LayerHole*> temp_set(this->holes.begin(),this->holes.end());//hole去重
    this->holes = vector<LayerHole*>(temp_set.begin(),temp_set.end());
    this->normal = getNormalFromOrientationTag(group->orientation_tag);
    this->unit_edges = getUnitEdgesFromHoles(this->holes);
    this->orientation_tag = group->orientation_tag;
    this->sketchGroup = group;

    std::sort(this->unit_edges.begin(), this->unit_edges.end(), [](UnitEdge* u1, UnitEdge* u2){ return u1->_id > u2->_id; } );

    std::sort(this->holes.begin(), this->holes.end(), [](LayerHole* h1, LayerHole* h2){ return h1->_id > h2->_id; });

    if(this->hole_num() == 1){
        this->component_type = Constants::TINY_COMPONENT;
    }else if(this->isConnectorComponent()){
        this->component_type = Constants::MINOR_COMPONENT;
    }else{
        this->component_type = Constants::MAJOR_COMPONENT;
    }
}


LayerHole* LayerGraph::getHole(int index)
{
    return holes.at(index);
}

bool LayerGraph::isConnectorComponent(){
    return this->sketchGroup->isConnectorComponent();
}

bool LayerGraph::isUniqueOrient() //whether the layergraph dont have a determined orientation yet
{
    return (this->orientation_tag == Constants::PLANE_YZ ||
            this->orientation_tag == Constants::PLANE_XY ||
            this->orientation_tag == Constants::PLANE_XZ);
}

int LayerGraph::getCurrentPlaneFlag(){//一定注意！！！在调用beam_search之前与之后是不同的值！！！所以在beam_search的时候使用的值与beamsearch之后的值不同
    //if(this->isConnectorComponent()){
      if( !this->isUniqueOrient() ){
        if(this->orientation_index%2==0)
            return std::get<0>(this->sketchGroup->getPossibleOriensAndCoord());
        else
            return std::get<1>(this->sketchGroup->getPossibleOriensAndCoord());
    }else{
        return this->orientation_tag;
    }
}

Eigen::Vector3d LayerGraph::getCurrentPlaneNormal(){
    int layer_tag = getCurrentPlaneFlag();
    switch (layer_tag){
        case Constants::PLANE_XY:{
            return Eigen::Vector3d(0,0,1);
            break;
        }
        case Constants::PLANE_YZ:{
            return Eigen::Vector3d(1,0,0);
            break;
        }
        case Constants::PLANE_XZ:{
            return Eigen::Vector3d(0,1,0);
            break;
        }
        default:{
            printf("layer flag error!! current flag : %d \n", layer_tag);
            exit(1);
        }
    }

}


bool LayerGraph::containSymmetryComponentsAllDirection(){
    return  !(this->sketchGroup->symm_group_x == NULL &&
              this->sketchGroup->symm_group_y == NULL &&
              this->sketchGroup->symm_group_z == NULL);
}

double LayerGraph::distance(LayerGraph* layer)// return the shortest distance between two graphs
{
    double shortest_dist = 999999.0;
    for(auto& h : this->holes){
        for(auto& h2 : layer->holes){
            double dist = (h->position()-h2->position()).norm();
            if( dist < shortest_dist){
                shortest_dist = dist;
            }
        }
    }
    return shortest_dist;
}

int LayerGraph::contactPointsNum(const LayerGraph& layer) //return the contacting number(number of connectors connect them) between two components.
{
    vector<tuple<LayerHole*,LayerHole*>> close_pairs;
    for(auto& h : this->holes){
        for(auto& h2 : layer.holes){
            if(fabs((h->position()-h2->position()).norm()-1)<0.05){
                close_pairs.push_back(std::make_tuple(h,h2));
            }
        }
    }

    close_pairs.erase(std::remove_if(close_pairs.begin(), close_pairs.end(),
                                     [](const tuple<LayerHole*,LayerHole*>& hole_pair)
                                     {
                                         auto lines = *glob_vars::sketchGraph->getLines();
                                         bool exsit = false;
                                         for(auto & line : lines){
                                             if (  line->contains(std::get<0>(hole_pair)->position()) &&
                                                   line->contains(std::get<1>(hole_pair)->position())
                                                )
                                                 exsit = true;
                                         }
                                         if(!exsit)
                                             return true;
                                         else
                                             return false;
                                     })
    ,close_pairs.end());

    return close_pairs.size();

}

//return if the hole sequence covered by b,(cover几个返回几个), 并且sequence是严格保证顺序与hole是一样
vector<LayerHole *> covered_hole_sequence(BeamTemplate &b, vector<LayerHole *> &all_holes) {
    vector<LayerHole*> result;
    vector<Vector3d> curr_comp_holes = b.getCurrentComponentHoles();

    //反向检查
    for(auto p = curr_comp_holes.cbegin(); p!=curr_comp_holes.cend();p++){
        bool exist = false;
        for (auto hole : all_holes){
            if ((*p - hole->position()).norm() < 0.05) {
                exist = true;
                result.push_back(hole);
                break;
            }
        }
        if(!exist)
            return result;
    }
    return result;
}

//Different from above method, this method is much efficient.
//TODO: this core do not support individual hole placement
void LayerGraph::pre_compute_all_placement(int plane_flag,
                                           const vector<BeamTemplate *> &gend_beams)//计算所有的可能的beam，将结果保存在hole中,没有layer
{
    /**
     * Delete previous result
     */
    for (BeamTemplate *b : this->all_possi_beams)
        delete b;
    this->all_possi_beams.clear();
    for(auto& h : holes){
        h->possible_beams.clear();
    }

    /*
     * Iterate each edge exactly once
     */
    BrickFactory brickFactory = BrickFactory();
    vector<BeamTemplate> init_beam_list = brickFactory.getAllBeams();
    Vector3d plane_normal;
    Matrix3d base_trans;
    Matrix3d ref_mat;

    switch(plane_flag){
        case Constants::PLANE_XY :
            plane_normal = Vector3d(0,0,1);
            base_trans = Eigen::AngleAxisd(M_PI/2, Vector3d(1, 0, 0)).matrix();
            ref_mat = Eigen::AngleAxisd(M_PI, Vector3d(0, 1, 0)).matrix();
            break;
        case Constants::PLANE_XZ :
            plane_normal = Vector3d(0,1,0);
            base_trans = Matrix3d::Identity(3,3);
            ref_mat = Eigen::AngleAxisd(M_PI, Vector3d(0, 0, 1)).matrix();
            break;
        case Constants::PLANE_YZ :
            plane_normal = Vector3d(1,0,0);
            base_trans = Eigen::AngleAxisd(M_PI/2, Vector3d(0, 0, 1)).matrix();
            ref_mat = Eigen::AngleAxisd(M_PI, Vector3d(0, 0, 1)).matrix();
            break;
        default:
            printf("Layer flag error!!!!!\n");
            exit(1);
    }


    set<LayerHole*, LayerHole::LayerHoleComp> visited_holes;
    for(auto & b : init_beam_list){

        if(b.holeNum() == 1) continue;
        int refNum = b.is_straight? 1 : 2;
        for(int ref = 0; ref < refNum ; ref++){
            for(auto & h : this->holes){
                for(auto & adj_h : h->adj_holes){

                    if(visited_holes.find(adj_h) == visited_holes.end()){//如果adj_h没有visit过
                        for(int p : {-1, 1}){//For each edge, try two orientation
                            b.transform(base_trans);
                            if(ref)
                                b.transform(ref_mat);
                            vector<Vector3d> curr_holes = b.getCurrentComponentHoles();
                            Vector3d beam_hole1 = curr_holes.at(0);//corresponding to hole h
                            Vector3d beam_hole2 = curr_holes.at(1);//corresponding to hole adj_h
                            //Calculate the angle to rotate
                            Vector3d orien_beam = beam_hole2 - beam_hole1;
                            Vector3d orien_layer_hole = p * (adj_h->position() - h->position());
                            Vector3d rot_axis = orien_beam.cross(orien_layer_hole);
                            if(rot_axis.norm()>0.005){//Not Same orientation
                                rot_axis = rot_axis/rot_axis.norm();
                                Matrix3d rot_mat = Eigen::AngleAxisd(acos(orien_beam.dot(orien_layer_hole)/(orien_beam.norm()*orien_layer_hole.norm())), rot_axis).matrix();
                                b.transform(rot_mat);
                            }else if((orien_beam-orien_layer_hole).norm()>0.005){//Opposite orientation
                                Matrix3d rot_mat = Eigen::AngleAxisd(M_PI, plane_normal).matrix();
                                b.transform(rot_mat);
                            }
                            b.translate(h->position() - b.getCurrentComponentHoles().at(p==1 ? 0 : 1));
                            vector<LayerHole*> hole_sequence = covered_hole_sequence(b, this->holes);
                            if(hole_sequence.size() == b.holeNum()){
                                //check existance, avoid duplication
                                bool exist = false;
                                for(auto & b_h : hole_sequence.front()->possible_beams){
                                    if(b.equal_on_cover_id(*b_h,hole_sequence)){
                                        exist = true;
                                        break;
                                    }
                                }
                                if(!exist)//说明没有重复
                                {
                                    BeamTemplate *new_beam = new BeamTemplate(b);
                                    new_beam->covered_edges = getUnitEdgesFromHoles(hole_sequence);
                                    new_beam->covered_holes = hole_sequence;

                                    //debugging:
                                    if( new_beam->covered_holes.size() != (new_beam->covered_edges.size() + 1) ){
                                        printf("warning: The current beam covers a non-sketch unit edge\n");
                                        continue;
                                    }

                                    all_possi_beams.push_back(new_beam);
                                    for(auto & ue : new_beam->covered_edges)
                                    {
                                        ue->possibleBeams.push_back(new_beam);
                                    }
                                    for(auto & s_hole : hole_sequence)
                                    {
                                        s_hole->possible_beams.push_back(new_beam);
                                    }
                                }
                            }
                            b.resetTransformation();
                        }
                    }
                }
                visited_holes.insert(h);
            }
            visited_holes.clear();
        }
        if(glob_vars::print_flag){
            printf("0 --------- pre-processing %s -------------------\n",b.ldraw_id.c_str());
            for(auto all_p_b : all_possi_beams){
                printf("0 STEP\n");
                all_p_b->toLDrawFormat();
            }
        }
    }

}


int LayerGraph::hole_num() const
{
    return holes.size();
}

int LayerGraph::beam_num() const
{
    return beams.size();
}

string LayerGraph::getCurrentHolesAsString(int color){
    std::ostringstream stringStream;
    printf("0 STEP ------------------Current holes on this graph -------------------------\n");
    for(int i = 0;i<holes.size();i++)
    {
        LayerHole* h = holes.at(i);
        stringStream<<h->toLdrawFormatAsString(color);
    }
    return stringStream.str();
}

/**
 * Establish adjacent relation, close relation and so on.
 */
void LayerGraph::establishAdjacentRelation(vector<SketchLine*>& sketchlines)
{
    for(auto & h : holes){
        h->adj_holes.clear();
        h->close_holes.clear();
    }
    for(int j = 0; j<hole_num();j++)
    {
        LayerHole* hole = getHole(j);
        for(int k = j+1; k<hole_num() ; k++)
        {
            LayerHole* adj_hole = getHole(k);
            if(fabs((hole->position()- adj_hole->position()).norm()-1.0)<0.08)
            {
                hole->add_adj_hole(adj_hole);
                adj_hole->add_adj_hole(hole);
            }
            if((hole->position()- adj_hole->position()).norm()< 0.98){
                hole->close_holes.insert(adj_hole);
                adj_hole->close_holes.insert(hole);
            }
        }
    }
//    check each pair of adjacent hole lis on a sketch line
    for(auto & h : this->holes){
        for(auto & adj_h : h->adj_holes){
            bool exsit = false;
            for(auto & sketch_line : sketchlines){
                if (sketch_line->contains(h->position()) && sketch_line->contains(adj_h->position()))
                    exsit = true;
            }
            if(!exsit){
                h->adj_holes.erase(adj_h);
                adj_h->adj_holes.erase(h);
                break;
            }
        }
    }
}

bool LayerGraph::establishSymmetryRelation()//此方法计算symmetry的关系并存储
{
    bool symmetry = false;
    if(glob_vars::current_model.symmetry_x &&(orientation_tag == Constants::PLANE_XZ || orientation_tag == Constants::PLANE_XY||
                                              orientation_tag == Constants::PLANE_XY_OR_XZ)){
        printf("current component is x-axis symmetry!\n");
        symmetry_X_General(holes, all_possi_beams);
        symmetry = true;
    }else if(glob_vars::current_model.symmetry_y &&(orientation_tag == Constants::PLANE_YZ || orientation_tag == Constants::PLANE_XY||
                                                    orientation_tag == Constants::PLANE_XY_OR_YZ)){
        printf("current component is y-axis symmetry!\n");
        symmetry_Y_General(holes, all_possi_beams);
        symmetry = true;
    }else if(glob_vars::current_model.symmetry_z &&(orientation_tag == Constants::PLANE_YZ || orientation_tag == Constants::PLANE_XZ||
                                                    orientation_tag == Constants::PLANE_YZ_OR_XZ)){
        printf("current component is z-axis symmetry!\n");
        symmetry_Z_General(holes,all_possi_beams);
        symmetry = true;
    } else{
        symmetry = false;
    }
    return symmetry;
}

void LayerGraph::erasePlaneSymmetryRelation(){

    if(glob_vars::current_model.symmetry_x &&(orientation_tag == Constants::PLANE_XZ || orientation_tag == Constants::PLANE_XY||
                                              orientation_tag == Constants::PLANE_XY_OR_XZ)){
        //如果两个hole在x方向是对称的，那么两个hole互为对方的symmetry hole
        for(auto& h1 : holes){
            for(auto& h2 : holes){
                if(fabs(h1->_x+h2->_x)<0.05 && fabs(h1->_y-h2->_y)<0.05 && fabs(h1->_z-h2->_z)<0.05){
                    auto symm_h1 = h1->getAllSymmetryHoles();
                    auto symm_h2 = h2->getAllSymmetryHoles();
                    for(auto& h : symm_h1) h->symmetry_hole_x = NULL;
                    for(auto& h : symm_h2) h->symmetry_hole_x = NULL;
                }
            }
        }

    }

    if(glob_vars::current_model.symmetry_y &&(orientation_tag == Constants::PLANE_YZ || orientation_tag == Constants::PLANE_XY||
                                                    orientation_tag == Constants::PLANE_XY_OR_YZ)){
        //如果两个hole在x方向是对称的，那么两个hole互为对方的symmetry hole
        for(auto& h1 : holes){
            for(auto& h2 : holes){
                if(fabs(h1->_x-h2->_x)<0.05 && fabs(h1->_y+h2->_y)<0.05 && fabs(h1->_z-h2->_z)<0.05){
                    auto symm_h1 = h1->getAllSymmetryHoles();
                    auto symm_h2 = h2->getAllSymmetryHoles();
                    for(auto& h : symm_h1) h->symmetry_hole_y = NULL;
                    for(auto& h : symm_h2) h->symmetry_hole_y = NULL;
                }
            }
        }

    }

    if(glob_vars::current_model.symmetry_z &&(orientation_tag == Constants::PLANE_YZ || orientation_tag == Constants::PLANE_XZ||
                                                    orientation_tag == Constants::PLANE_YZ_OR_XZ)){
        //如果两个hole在x方向是对称的，那么两个hole互为对方的symmetry hole
        for(auto& h1 : holes){
            for(auto& h2 : holes){
                if(fabs(h1->_x-h2->_x)<0.05 && fabs(h1->_y-h2->_y)<0.05 && fabs(h1->_z+h2->_z)<0.05){
                    auto symm_h1 = h1->getAllSymmetryHoles();
                    auto symm_h2 = h2->getAllSymmetryHoles();
                    for(auto& h : symm_h1) h->symmetry_hole_z = NULL;
                    for(auto& h : symm_h2) h->symmetry_hole_z = NULL;
                }
            }
        }
    }


}

void LayerGraph::establishSymmetryUnitEdgeRelation()//计算unit edge的symmetry关系并存储
{
    for(auto iter = unit_edges.begin(); iter != unit_edges.end(); iter++){
        for(auto iter2 = iter +1; iter2 != unit_edges.end(); iter2++){
            if(((*iter)->hole1->symmtry_hole == (*iter2)->hole1 || (*iter)->hole1->symmtry_hole == (*iter2)->hole2) &&
               ((*iter)->hole2->symmtry_hole == (*iter2)->hole1 || (*iter)->hole2->symmtry_hole == (*iter2)->hole2)){//说明两个unit edge基于layer hole是symmetry的
                (*iter)->symm_uedge  = (*iter2);
                (*iter2)->symm_uedge = (*iter);
            }
        }
    }
}

void LayerGraph::perm_layer(int num_slots, int max_layer, int* result, int beam_size)
{
    for(int i = 0; i < beam_size;i++)
        printf("%d ",result[i]);
    printf("\n");

//    if(!is_connect(result)){
//        return;
//    }
    if(num_slots == beam_size){
        //printCurrentBeams();
    }else{
        for(int i = 0;i<max_layer;i++)
        {
            result[num_slots] = i;
            perm_layer(num_slots+1,max_layer,result,beam_size);
        }
    }
}

void LayerGraph::pre_compute_single_holes(int m_plane_flag) //add single-hole solution to the possible beams
{
    vector<Vector3d> tb1_holes = { Vector3d(0,0,0) };
    BeamTemplate TB1("TB1", tb1_holes, "18654.dat", Vector3d(0, 1, 0), true);
    Matrix3d base_trans;
    switch(m_plane_flag){
        case Constants::PLANE_XY :
            base_trans = Eigen::AngleAxisd(M_PI/2, Vector3d(1, 0, 0)).matrix();
            break;
        case Constants::PLANE_XZ :
            base_trans = Matrix3d::Identity(3,3);
            break;
        case Constants::PLANE_YZ :
            base_trans = Eigen::AngleAxisd(M_PI/2, Vector3d(0, 0, 1)).matrix();
            break;
        default:
            printf("Layer flag error!!!!!\n");
            exit(0);
    }
    TB1.transform(base_trans);

    for (const auto& hole : this->holes) {
        BeamTemplate *unit_beam = new BeamTemplate(TB1);
        unit_beam->translate(hole->position());
        unit_beam->layer = 0;
        unit_beam->color = 7;
        all_possi_beams.push_back(unit_beam);
        hole->possible_beams.push_back(unit_beam);
        unit_beam->covered_holes.push_back(hole);
    }

}

//The goal of this function is add all the direct or undirect connected holes with h to holes
void all_adj_holes(LayerHole* h, set<LayerHole*>& holes){
    holes.insert(h);
    for(const auto & adj_h : h->adj_holes){
        if(holes.find(adj_h) == holes.end())//If didn't contains then insert it
        {
            all_adj_holes(adj_h,holes);
        }
    }
}

void LayerGraph::updateBeamOrientation(){

    int curr_flag = getCurrentPlaneFlag();
    Vector3d curr_normal = getCurrentPlaneNormal();

    for(auto& b : all_possi_beams){
        Vector3d beamNormal = b->getNormal();
        if( fabs(beamNormal.dot(curr_normal)) < 5e-2 ){
            Vector3d rot_axis = beamNormal.cross(curr_normal);
            Matrix3d base_trans = Eigen::AngleAxisd(M_PI/2, rot_axis).matrix();
            b->transform(base_trans);
        }
    }
    this->normal = curr_normal;
}
