//
// Created by student on 4/24/2019.
//

#ifndef LEGO_TECHNIC_MAIN_MYASSERT_H
#define LEGO_TECHNIC_MAIN_MYASSERT_H

namespace MyAsserts{

    bool NoRepeatSketchPoints(vector<SketchPoint*>& points);

    ////assert p1 and p2 are far away as "distance"
    bool isDistance(const Vector3d & p1, const Vector3d & p2, double distance);
};

#endif //LEGO_TECHNIC_MAIN_MYASSERT_H
