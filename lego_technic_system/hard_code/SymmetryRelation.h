//
// Created by 徐豪 on 2018/4/19.
//

#ifndef LEGOGENERATION_PLANE_GENERATION2_SYMMETRYRELATION_H
#define LEGOGENERATION_PLANE_GENERATION2_SYMMETRYRELATION_H

#include <vector>
#include "core/LayerHole.h"


void symmetry_X_General(std::vector<LayerHole *> m_holes, vector<BeamTemplate *> m_beams);

void symmetry_Y_General(std::vector<LayerHole *> m_holes, vector<BeamTemplate *> m_beams);

void symmetry_Z_General(std::vector<LayerHole *> m_holes, vector<BeamTemplate *> m_beams);




#endif //LEGOGENERATION_PLANE_GENERATION2_SYMMETRYRELATION_H
