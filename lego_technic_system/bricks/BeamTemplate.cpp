//
// Created by 徐豪 on 2017/6/3.
//

#include "BeamTemplate.h"
#include "util/util.h"
#include <set>
#include "../core/LayerGraph.h"
#include "Eigen/Dense"

using std::to_string;
using std::string;
using std::cout;
using std::endl;

int BeamTemplate::max_id = 0;


BeamTemplate::BeamTemplate(string name, vector<Vector3d> component_holes, string LD_id, Vector3d normal,
                           bool is_straight, int color) :
        name(name),component_holes(component_holes),ldraw_id(LD_id),normal(normal),color(color),is_straight(is_straight)
{
    this->layer = 0;
    this->feature_beam = false;
    this->refl_symmetry_beam = NULL;
    resetTransformation();
}

BeamTemplate::BeamTemplate(const BeamTemplate &b) :
        color(b.color), name(b.name), ldraw_id(b.ldraw_id), layer(b.layer), local_score(b.local_score), covered_holes(b.covered_holes),
        axle_index(b.axle_index), transf_mat(b.transf_mat), translation(b.translation), normal(b.normal),
        component_holes(b.component_holes),
        feature_beam(b.feature_beam), refl_symmetry_beam(b.refl_symmetry_beam), sub_beams(b.sub_beams),
        is_straight(is_straight)
{
    this->_id = max_id++;
    this->is_straight = b.is_straight;
}

BeamTemplate &BeamTemplate::transform(const Matrix3d &mat)
{
    transf_mat = mat * transf_mat;
    return *this;
}

BeamTemplate &BeamTemplate::translate(const Vector3d &trans)
{
    translation = translation + trans;
    return *this;
}

void BeamTemplate::resetTransformation()
{
    this->transf_mat = Matrix3d::Identity(3,3);
    this->translation = Vector3d(0,0,0);
}

vector<Vector3d> BeamTemplate::getCurrentComponentHoles() const
{
    vector<Vector3d> vec;
    Vector3d current_normal = transf_mat * normal;
    if(fabs(fabs(current_normal.dot(Vector3d(1,0,0)))-1)<0.05){
        current_normal = Vector3d(1,0,0);
    }else if(fabs(fabs(current_normal.dot(Vector3d(0,1,0)))-1)<0.05){
        current_normal = Vector3d(0,1,0);
    }else if(fabs(fabs(current_normal.dot(Vector3d(0,0,1)))-1)<0.05){
        current_normal = Vector3d(0,0,1);
    }else{
        printf("Unorthorgnal Normal!!!\n");
    }
    for(const Vector3d& hole:component_holes)
    {
        vec.push_back(transf_mat * hole + translation + layer * current_normal);
    }
    return vec;
}

int BeamTemplate::holeNum() const
{
    return this->component_holes.size();
}

Vector3d BeamTemplate::getNormal() const
{
    return transf_mat * normal;
}

void BeamTemplate::print() const
{
    printf("BeamName:%s",name.c_str());
    for(auto hole : getCurrentComponentHoles())
    {
        Vector3d v = transf_mat * hole + translation;
        printf("(%.1lf,%.1lf,%.1lf) ",v.x(),v.y(),v.z());
    }
    printf("\n");
}

set<int> BeamTemplate::getFeatureHoleID() const {
    assert(!this->covered_holes.empty());
    set<int> result;
    result.insert(this->covered_holes.front()->_id);
    result.insert(this->covered_holes.back()->_id);
    return result;
}

set<LayerHole *> BeamTemplate::getFeatureHole() const {
    assert(!this->covered_holes.empty());
    set<LayerHole*> result;
    result.insert(this->covered_holes.front());
    result.insert(this->covered_holes.back());
    return result;
}


void BeamTemplate::toLDrawFormat() const {
    Vector3d current_normal = transf_mat * normal;
    if(fabs(fabs(current_normal.dot(Vector3d(1,0,0)))-1)<0.05){
        current_normal = Vector3d(1,0,0);
    }else if(fabs(fabs(current_normal.dot(Vector3d(0,1,0)))-1)<0.05){
        current_normal = Vector3d(0,1,0);
    }else if(fabs(fabs(current_normal.dot(Vector3d(0,0,1)))-1)<0.05){
        current_normal = Vector3d(0,0,1);
    }else{
        printf("Unorthorgnal Normal!!!\n");
        cin.get();
    }
    if(abs(layer)>10) {
        printf("Beam layer number error!!! %d\n",layer);
        cin.get();
    }
    Vector3d pos = translation + layer * current_normal;
    printf("1 %d %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %s\n",color,
           20*pos.x(),20*pos.y(),20*pos.z(),
           transf_mat(0,0),transf_mat(0,1),transf_mat(0,2),
           transf_mat(1,0),transf_mat(1,1),transf_mat(1,2),
           transf_mat(2,0),transf_mat(2,1),transf_mat(2,2),
           ldraw_id.c_str());
}

void BeamTemplate::toLDrawFormat(int m_color) const {
    Vector3d current_normal = transf_mat * normal;

    if(fabs(fabs(current_normal.dot(Vector3d(1,0,0)))-1)<0.05){
        current_normal = Vector3d(1,0,0);
    }else if(fabs(fabs(current_normal.dot(Vector3d(0,1,0)))-1)<0.05){
        current_normal = Vector3d(0,1,0);
    }else if(fabs(fabs(current_normal.dot(Vector3d(0,0,1)))-1)<0.05){
        current_normal = Vector3d(0,0,1);
    }else{
        printf("Unorthorgnal Normal!!!\n");
    }

    Vector3d pos = translation + layer * current_normal;
    printf("1 %d %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %s\n",m_color,
           20*pos.x(),20*pos.y(),20*pos.z(),
           transf_mat(0,0),transf_mat(0,1),transf_mat(0,2),
           transf_mat(1,0),transf_mat(1,1),transf_mat(1,2),
           transf_mat(2,0),transf_mat(2,1),transf_mat(2,2),
           ldraw_id.c_str());
}

string BeamTemplate::toLDrawFormatAsString(int m_color) const {
    Vector3d current_normal = transf_mat * normal;
    if(fabs(fabs(current_normal.dot(Vector3d(1,0,0)))-1)<0.05){
        current_normal = Vector3d(1,0,0);
    }else if(fabs(fabs(current_normal.dot(Vector3d(0,1,0)))-1)<0.05){
        current_normal = Vector3d(0,1,0);
    }else if(fabs(fabs(current_normal.dot(Vector3d(0,0,1)))-1)<0.05){
        current_normal = Vector3d(0,0,1);
    }else{
        printf("0 Unorthorgnal Normal!!!\n");
    }
    Vector3d pos = translation + layer * current_normal;
    char buff[200];
    snprintf(buff, sizeof(buff), "1 %d %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %s\n",color,
             20*pos.x(),20*pos.y(),20*pos.z(),
             transf_mat(0,0),transf_mat(0,1),transf_mat(0,2),
             transf_mat(1,0),transf_mat(1,1),transf_mat(1,2),
             transf_mat(2,0),transf_mat(2,1),transf_mat(2,2),
             ldraw_id.c_str());
    std::string buffAsStdStr = buff;
    return buffAsStdStr;
}

string BeamTemplate::toIndividualLDrawAsString(int color) const {
    if(this->sub_beams.empty()){
        return toLDrawFormatAsString(color);
    }else{
        Vector3d current_normal = transf_mat * normal;
        if(fabs(fabs(current_normal.dot(Vector3d(1,0,0)))-1)<0.05){
            current_normal = Vector3d(1,0,0);
        }else if(fabs(fabs(current_normal.dot(Vector3d(0,1,0)))-1)<0.05){
            current_normal = Vector3d(0,1,0);
        }else if(fabs(fabs(current_normal.dot(Vector3d(0,0,1)))-1)<0.05){
            current_normal = Vector3d(0,0,1);
        }else{
            printf("0 Unorthorgnal Normal!!!\n");
        }
        Vector3d pos = translation + layer * current_normal;
        string result;
        for(auto b : sub_beams){
            b.translation = pos + transf_mat * b.translation;
            b.transform(this->transf_mat);
            result += b.toLDrawFormatAsString(color);
        }
        return result;
    }
}

bool BeamTemplate::operator==(const BeamTemplate &b)
{
    if((this->name != b.name) || (this->ldraw_id != b.ldraw_id))
        return false;
    else{
        vector<Vector3d> comp1 = this->getCurrentComponentHoles();
        vector<Vector3d> comp2 = b.getCurrentComponentHoles();
        if(comp1.size()!=comp2.size())
            return false;
        else{
            for(int i = 0;i<comp1.size();i ++)
            {
                int j = 0;
                for(;j < comp2.size(); j++)
                {
                    if((comp1.at(i) - comp2.at(j)).norm()<0.05)
                        break;
                }
                if (j==comp1.size())
                    return false;
            }
            return true;
        }
    }
}

bool BeamTemplate::equal_on_cover_id(const BeamTemplate &b, vector<LayerHole *> hole_sequence) {
    using namespace std;
    if((this->name != b.name) || (this->ldraw_id != b.ldraw_id) || (this->holeNum()!=b.holeNum()))
        return false;
    else{
        set<int> id_features = hole_sequence.empty()? this->getFeatureHoleID()
                                                  :set<int>{hole_sequence.front()->_id,hole_sequence.back()->_id};
        if(id_features == b.getFeatureHoleID()){
            return true;
        }else{
            return false;
        }
    }
}

double BeamTemplate::distance(LayerHole *h)//返回这个beam与给定hole的最近距离
{
    double shortest = 999999999.0;
    for(auto & b_h : this->covered_holes){
        if((b_h->position()-h->position()).norm()<shortest)
            shortest = (b_h->position()-h->position()).norm();
    }
    return shortest;
}

bool BeamTemplate::isSymmetry(BeamTemplate &b)
{
    using namespace std;
    if((this->name != b.name) || (this->ldraw_id != b.ldraw_id) || (this->holeNum()!=b.holeNum()))
        return false;
    else{
        set<LayerHole*> feature_holes1 = this->getFeatureHole();
        set<LayerHole*> feature_holes2 = b.getFeatureHole();
        for(auto f_hole : feature_holes1){
            if(feature_holes2.find(f_hole->symmtry_hole)==feature_holes2.end())
                return false;
        }
    }
    return true;
}
