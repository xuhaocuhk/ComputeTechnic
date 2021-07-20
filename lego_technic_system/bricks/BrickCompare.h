//
// Created by 徐豪 on 2018/7/1.
//

#ifndef LEGO_TECHNIC_MAIN_BRICKCOMPARE_H
#define LEGO_TECHNIC_MAIN_BRICKCOMPARE_H

#include "BeamInstance.h"

struct BeamComp{
    bool operator () (const BeamInstance& b1, const BeamInstance& b2) const
    {
        return b1.template_beam->_id < b2.template_beam->_id;
    }
};

#endif //LEGO_TECHNIC_MAIN_BRICKCOMPARE_H
