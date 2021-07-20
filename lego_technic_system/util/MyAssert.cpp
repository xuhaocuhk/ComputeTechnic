//
// Created by student on 4/24/2019.
//

#include <core/SketchPoint.h>
#include "MyAssert.h"
#include <iostream>

using namespace std;
/***** checking repeating point in the file ******/
bool MyAsserts::NoRepeatSketchPoints(vector<SketchPoint*>& points){

    for(auto p1 = points.begin(); p1 != points.end(); p1++ ){
        for(auto p2 = p1 + 1; p2 != points.end(); p2++ ){
            if( fabs(((*p1)->getPointVec() - (*p2)->getPointVec()).norm())<0.2 )
            {
                printf("reduldant point in the input file!!!\n");
                cout<< (*p1)->getPointVec() << endl;
                cout<< (*p2)->getPointVec() << endl;
                return false;
            }
        }
    }

    return true;
}

bool MyAsserts::isDistance(const Vector3d & p1, const Vector3d & p2, double distance){
    return fabs(fabs((p1 - p2).norm()) - distance ) < 1e-2;
}