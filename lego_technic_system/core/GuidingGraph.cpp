//
// Created by student on 4/24/2019.
//

#include <configure/global_variables.h>
#include "GuidingGraph.h"
#include "IO/DataWriter.h"
#include "util/util.h"
#include "util/MyAssert.h"

GuidingGraph::GuidingGraph(SketchGraph* sketchGraph) {

    for(auto& l : *sketchGraph->m_lines){
        Vector3d p1 = l->_p1->getPointVec();
        Vector3d p2 = l->_p2->getPointVec();
        int length = l->intLength();
        double step = (double)1/length;
        vector<LayerHole*> result_holes;

        /**** Interior holes *****/
        for(double t = -1e-8 + step; t <= 1 + 1e-8 - step; t+=step)
        {
            Vector3d pos = p1 + t * (p2-p1);
            LayerHole* hole = new LayerHole(pos.x(), pos.y(), pos.z(), Constants::UNKNOWN);
            hole->featured = l->feature_line;
            result_holes.push_back(hole);
        }

        /**** Two End points ****/
        LayerHole* hole_p1 = l->_p1->layerhole == NULL ?
                             new LayerHole(l->_p1->_x, l->_p1->_y, l->_p1->_z, Constants::UNKNOWN) :
                             l->_p1->layerhole;
        l->_p1->layerhole = hole_p1;
        hole_p1->sketchPoint = l->_p1;
        hole_p1->featured = l->feature_line;
        result_holes.insert(result_holes.begin(),hole_p1);

        //extract end point 2
        LayerHole* hole_p2 = l->_p2->layerhole == NULL ?
                             new LayerHole(l->_p2->_x, l->_p2->_y, l->_p2->_z, Constants::UNKNOWN) :
                             l->_p2->layerhole;
        l->_p2->layerhole = hole_p2;
        hole_p2->sketchPoint = l->_p2;
        hole_p2->featured = l->feature_line;
        result_holes.push_back(hole_p2);

        /**** Generate unit edges from every two holes ****/
        for(auto iter = result_holes.begin(); (iter+1)!=result_holes.end(); iter++){
            assert( MyAsserts::isDistance( (*iter)->position(), (*(iter+1))->position(), 1.0) );
            UnitEdge* u_edge = new UnitEdge(l, *iter, *(iter+1));
            (*iter)->unit_edges.insert(u_edge);
            (*(iter+1))->unit_edges.insert(u_edge);
            l->unit_edges.push_back(u_edge);
        }
    }

    //store unit edges
    for(auto& l : *sketchGraph->m_lines){
        for(auto& ue : l->unit_edges){
            u_edges.insert(ue);
            m_holes.insert(ue->hole1);
            m_holes.insert(ue->hole2);
        }
    }

    /***** Checking for repeating ******/
    for(auto& h1 : m_holes){
        for(auto& h2 : m_holes){
            if(h1 == h2) continue;

            if( MyAsserts::isDistance(h1->position() , h2->position(), 0.0 ) ){
                std::cout<<h1->position()<<endl;
                std::cout<<h2->position()<<endl;
                printf("repeating hole!!!!\n");
                cin.get();
            }
        }
    }

    this->establishLayerHoleSymmetry();
    this->establishUnitEdgeSymmetry();

    DataWriter::writeUnitEdges(u_edges, m_holes, "whole");
}


void GuidingGraph::establishLayerHoleSymmetry(){
    for(auto& h1 : m_holes){
        for(auto& h2 : m_holes){
            if(h1 == h2) continue;

            if (glob_vars::current_model.symmetry_x && fabs(( h1->position() + h2->position()).x()) < 1e-2
                && fabs(( h1->position() - h2->position()).y()) < 1e-2
                && fabs(( h1->position() - h2->position()).z()) < 1e-2){
                h1->symmetry_hole_x = h2;
                h2->symmetry_hole_x = h1;
            }

            if (glob_vars::current_model.symmetry_y && fabs(( h1->position() + h2->position()).y()) < 1e-2
                && fabs(( h1->position() - h2->position()).x()) < 1e-2
                && fabs(( h1->position() - h2->position()).z()) < 1e-2){
                h1->symmetry_hole_y = h2;
                h2->symmetry_hole_y = h1;
            }

            if (glob_vars::current_model.symmetry_z && fabs(( h1->position() + h2->position()).z()) < 1e-2
                && fabs(( h1->position() - h2->position()).x()) < 1e-2
                && fabs(( h1->position() - h2->position()).y()) < 1e-2){
                h1->symmetry_hole_z = h2;
                h2->symmetry_hole_z = h1;
            }

        }
    }
}

void GuidingGraph::establishUnitEdgeSymmetry(){

    for(auto& ue1 : u_edges){
        for(auto& ue2 : u_edges){
            if(ue1 == ue2) continue;

            LayerHole* ue1_p1 = ue1->hole1;
            LayerHole* ue1_p2 = ue1->hole2;

            Vector3d p1_ref_x = ue1_p1->position(); p1_ref_x.x() *= -1.0;
            Vector3d p2_ref_x = ue1_p2->position(); p2_ref_x.x() *= -1.0;
            Vector3d p1_ref_y = ue1_p1->position(); p1_ref_y.y() *= -1.0;
            Vector3d p2_ref_y = ue1_p2->position(); p2_ref_y.y() *= -1.0;
            Vector3d p1_ref_z = ue1_p1->position(); p1_ref_z.z() *= -1.0;
            Vector3d p2_ref_z = ue1_p2->position(); p2_ref_z.z() *= -1.0;

            Vector3d l_p1 = ue2->hole1->position();
            Vector3d l_p2 = ue2->hole2->position();
            if(glob_vars::current_model.symmetry_x && isSameSegment(l_p1,l_p2, p1_ref_x, p2_ref_x)){
                ue1->symm_uedge_x = ue2;
                ue2->symm_uedge_x = ue1;
            }
            if(glob_vars::current_model.symmetry_y && isSameSegment(l_p1,l_p2, p1_ref_y, p2_ref_y)){
                ue1->symm_uedge_y = ue2;
                ue2->symm_uedge_y = ue1;
            }
            if(glob_vars::current_model.symmetry_z && isSameSegment(l_p1,l_p2, p1_ref_z, p2_ref_z)){
                ue1->symm_uedge_z = ue2;
                ue2->symm_uedge_z = ue1;
            }

        }
    }
}