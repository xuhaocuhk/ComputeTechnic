//
// Created by 徐豪 on 2018/6/28.
//

#ifndef LEGO_TECHNIC_MAIN_BEAMINSTANCE_H
#define LEGO_TECHNIC_MAIN_BEAMINSTANCE_H

#include <bricks/BeamTemplate.h>
#include <configure/Constants.h>
#include "core/LayerGraph.h"

class BeamTemplate;

using std::string;

//代表一个beam的实例，其背后是一个possible placement
class BeamInstance {

public:

    struct BeamInstanceComp {
        bool operator() (BeamInstance* b1, BeamInstance* b2) const
        {
            if( b1->template_beam->_id < b2->template_beam->_id ){
                return true;
            }else if (b1->template_beam->_id > b2->template_beam->_id ){
                return false;
            }else{
                return b1->layer_number < b2->layer_number;
            }
        }
    };


public:
    BeamInstance(BeamTemplate *);

    bool operator<(const BeamInstance & beam)const;   //重载<运算符
    BeamTemplate *getBeam();//得到具有相同property的beam

    string toLdrawFormatAsString();

public:
    BeamTemplate *template_beam = NULL;
    int layer_number = 0;
    int orientation = Constants::UNKNOWN;
    int color = 15;
    BeamInstance* symm_beam = NULL;
    vector<BeamHole*> beamholes; // number of beamholes may be bigger then the number of real holes due to symmetry beam
};


#endif //LEGO_TECHNIC_MAIN_BEAMINSTANCE_H
