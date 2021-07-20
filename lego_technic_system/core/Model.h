//
// Created by 徐豪 on 2018/4/17.
//

#ifndef LEGOGENERATION_PLANE_GENERATION2_INPUT_H
#define LEGOGENERATION_PLANE_GENERATION2_INPUT_H

#include <string>
#include <iostream>
#include <vector>

struct model{
    std::string name;
    std::string sketch_path;
    bool symmetry_x; //如果是symmetry的需要下面的函数建立symmetry关系
    bool symmetry_y;
    bool symmetry_z;
    int seed_number;
    double cool_rate;
    std::vector<double> generating_weight;

    std::string getDirectoryName(){
        return sketch_path.substr(0,sketch_path.find_first_of('/',0));
    }
};


#endif //LEGOGENERATION_PLANE_GENERATION2_INPUT_H