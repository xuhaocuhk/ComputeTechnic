//
// Created by 徐豪 on 2017/11/18.
//

#ifndef LEGOGENERATION_MAIN_PIN_H
#define LEGOGENERATION_MAIN_PIN_H

#include "Eigen/Dense"

using Eigen::Vector3d;
using Eigen::Matrix3d;

class Pin {
public:
    Pin(Vector3d position,Vector3d orientation,bool isAxle,bool insertable);

    Vector3d pos;
    Vector3d orientation;//pin的朝向，(0,0,0)表示不insertable
    bool isAxle;
    bool insertable;
};


#endif //LEGOGENERATION_MAIN_PIN_H
