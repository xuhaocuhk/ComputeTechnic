//
// Created by 徐豪 on 2018/4/22.
//
#include "SymmetryRelation.h"


void symmetry_X_General(std::vector<LayerHole *> m_holes, vector<BeamTemplate *> m_beams) {
    //如果两个hole在x方向是对称的，那么两个hole互为对方的symmetry hole
    for(auto& h1 : m_holes){
        if(h1->symmtry_hole == NULL){
            for(auto& h2 : m_holes){
                if(fabs(h1->_x+h2->_x)<0.05 && fabs(h1->_y-h2->_y)<0.05 && fabs(h1->_z-h2->_z)<0.05){
                    h1->symmtry_hole = h2;
                    h2->symmtry_hole = h1;
                }
            }
        }
    }

    //Establish symmetry beam relation
    for(auto& b : m_beams){
        if(b->refl_symmetry_beam == NULL){
            for(auto& b2 : m_beams){
                if(b->isSymmetry(*b2)){
                    b->refl_symmetry_beam = b2;
                    b2->refl_symmetry_beam= b;
                    break;
                }
            }
        }
    }
}

void symmetry_Y_General(std::vector<LayerHole *> m_holes, vector<BeamTemplate *> m_beams) {
    //如果两个hole在x方向是对称的，那么两个hole互为对方的symmetry hole
    for(auto& h1 : m_holes){
        if(h1->symmtry_hole == NULL){
            for(auto& h2 : m_holes){
                if(fabs(h1->_x-h2->_x)<0.05 && fabs(h1->_y+h2->_y)<0.05 && fabs(h1->_z-h2->_z)<0.05){
                    h1->symmtry_hole = h2;
                    h2->symmtry_hole = h1;
                }
            }
        }
    }

    //Establish symmetry beam relation
    for(auto& b : m_beams){
        if(b->refl_symmetry_beam == NULL){
            for(auto& b2 : m_beams){
                if(b->isSymmetry(*b2)){
                    b->refl_symmetry_beam = b2;
                    b2->refl_symmetry_beam= b;
                    break;
                }
            }
        }
    }
}


void symmetry_Z_General(std::vector<LayerHole *> m_holes, vector<BeamTemplate *> m_beams) {
    //如果两个hole在x方向是对称的，那么两个hole互为对方的symmetry hole
    for(auto& h1 : m_holes){
        if(h1->symmtry_hole == NULL){
            for(auto& h2 : m_holes){
                if(fabs(h1->_x-h2->_x)<0.05 && fabs(h1->_y-h2->_y)<0.05 && fabs(h1->_z+h2->_z)<0.05){
                    h1->symmtry_hole = h2;
                    h2->symmtry_hole = h1;
                }
            }
        }
    }

    //Establish symmetry beam relation
    for(auto& b : m_beams){
        if(b->refl_symmetry_beam == NULL){
            for(auto& b2 : m_beams){
                if(b->isSymmetry(*b2)){
                    b->refl_symmetry_beam = b2;
                    b2->refl_symmetry_beam= b;
                    break;
                }
            }
        }
    }
}

