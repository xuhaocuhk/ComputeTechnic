//
// Created by 徐豪 on 2018/6/28.
//

#include <util/Helper.h>
#include "BeamInstance.h"

BeamInstance::BeamInstance(BeamTemplate *beam)
{
    this->template_beam = beam;
    this->layer_number = NO_LAYER;
}

bool BeamInstance::operator<(const BeamInstance & beam)const
{
    return this->template_beam->_id < beam.template_beam->_id;
}

BeamTemplate *BeamInstance::getBeam()//得到具有相同property的beam
{
    BeamTemplate *b = new BeamTemplate(*this->template_beam);
    b->layer = this->layer_number;
    return b;
}

string BeamInstance::toLdrawFormatAsString(){
    Vector3d current_normal = template_beam->transf_mat * template_beam->normal;
    if(fabs(fabs(current_normal.dot(Vector3d(1,0,0)))-1)<0.05){
        current_normal = Vector3d(1,0,0);
    }else if(fabs(fabs(current_normal.dot(Vector3d(0,1,0)))-1)<0.05){
        current_normal = Vector3d(0,1,0);
    }else if(fabs(fabs(current_normal.dot(Vector3d(0,0,1)))-1)<0.05){
        current_normal = Vector3d(0,0,1);
    }else{
        printf("Unorthorgnal Normal!!!\n");
    }
    Vector3d pos = template_beam->translation + this->layer_number * current_normal;
    char buff[200];
    snprintf(buff, sizeof(buff), "1 %d %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %s\n", this->color,
             20*pos.x(),20*pos.y(),20*pos.z(),
             template_beam->transf_mat(0,0), template_beam->transf_mat(0,1), template_beam->transf_mat(0,2),
             template_beam->transf_mat(1,0), template_beam->transf_mat(1,1), template_beam->transf_mat(1,2),
             template_beam->transf_mat(2,0), template_beam->transf_mat(2,1), template_beam->transf_mat(2,2),
             template_beam->ldraw_id.c_str());
    std::string buffAsStdStr = buff;
    return buffAsStdStr;
}