//
// Created by 徐豪 on 2018/4/28.
//

#ifndef LEGOGENERATION_MAIN_BOUNDINGBOX_H
#define LEGOGENERATION_MAIN_BOUNDINGBOX_H

#include <vector>
#include "bricks/BeamTemplate.h"

bool inBoundingBox(vector<BeamTemplate *> &);
bool inBoundingBox(vector<Connector*>&);
bool inBoundingBox(const Connector&);

#endif //LEGOGENERATION_MAIN_BOUNDINGBOX_H
