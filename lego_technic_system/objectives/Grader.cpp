//
// Created by 徐豪 on 2017/11/3.
//

#include <util/util.h>
#include <util/Symmetry.h>
#include "configure/global_variables.h"
#include "Grader.h"
#include "util/Helper.h"
#include "IO/DataWriter.h"

double Grader::deviation2sketch(BeamLayout& beamLayout){
    int total_dev = 0;
    int total_hole_num = 0;
    for(auto& b : beamLayout.beams){
        if(b->layer_number == NO_LAYER) continue;
        total_dev += ( b->template_beam->holeNum() * (b->layer_number * b->layer_number) );
        total_hole_num += b->template_beam->holeNum();
    }
    assert(total_hole_num > 0);
    return (double)total_dev/total_hole_num;
}

double Grader::deviation2sketch_online( BeamLayout& beamLayout, GradeRecorder& gradeRecorder )//locally update version, update gradeRecorder outside
{

    return gradeRecorder.dev_old_total_dev / gradeRecorder.dev_old_beamhole_num;

}

double Grader::deviation2sketch_online_3D(ConstructionGraph& constructionGraph){
    double total_dev = 0.0;
    int    total_beamNum = 0;
    for(auto& comp : constructionGraph.components){
        int symm_group_num = comp->layerGraph->sketchGroup->getAllSymmetryGroups().size();
        total_dev       += comp->gradeRecorder.dev_old_total_dev * symm_group_num;
        total_beamNum   += comp->gradeRecorder.simp_old_total_beamhole_num * symm_group_num;
    }

    assert(total_beamNum != 0);
    double deviation_score_3d = total_dev / total_beamNum;

//    printf("Total_dev: %.2lf, Total_holeNum: %d , Total: %lf \n", total_dev, total_beamNum, deviation_score_3d);

    return deviation_score_3d;
}

double Grader::deviation2sketch_3D(ConstructionGraph& constructionGraph){
    int total_dev = 0;
    int total_hole_num = 0;
    for(auto& comp : constructionGraph.components){
        int symm_group_num = comp->layerGraph->sketchGroup->getAllSymmetryGroups().size();
        for(auto& b : comp->beamLayout->beams){
            if(b->layer_number == NO_LAYER) continue;
            total_dev      += ( b->template_beam->holeNum() * (b->layer_number * b->layer_number) ) * symm_group_num ;
            total_hole_num += b->template_beam->holeNum() * symm_group_num;
        }
    }

    assert(total_hole_num > 0);
    return (double)total_dev/total_hole_num;
}

//越低越好
double Grader::simplicity(BeamLayout& beamLayout){
    set<LayerHole*> covered_holes;
    int total_beamhole_num = 0;
    for(auto& beam : beamLayout.beams){
        if(beam->layer_number == NO_LAYER) continue;
        covered_holes.insert(beam->template_beam->covered_holes.begin(), beam->template_beam->covered_holes.end());
        total_beamhole_num += beam->template_beam->holeNum();
    }
    double total_score = (double)total_beamhole_num / (double)covered_holes.size();
    assert(total_score>=0.999 && total_score < 9999.0);
    return total_score;
}

double Grader::simplicity_online(BeamLayout & beamLayout, GradeRecorder& gradeRecorder) {

    return (double)gradeRecorder.simp_old_total_beamhole_num / gradeRecorder.beamNumOnHole.size();

}

double Grader::simplicity_3D(ConstructionGraph& constructionGraph){
    set<LayerHole*> covered_holes;
    int total_coverd_holes_num = 0;
    int total_beamhole_num     = 0;

    for(auto& comp : constructionGraph.components){

        int symm_group_num = comp->layerGraph->sketchGroup->getAllSymmetryGroups().size();
        int holenum_before_comp = covered_holes.size(); // number of holes that before precessing this component

        for(auto& beam : comp->beamLayout->beams){
            if(beam->layer_number == NO_LAYER) continue;
            covered_holes.insert(beam->template_beam->covered_holes.begin(), beam->template_beam->covered_holes.end());
            total_beamhole_num += beam->template_beam->holeNum() * symm_group_num ;
        }
        // counting for the symmetry components
        total_coverd_holes_num += (covered_holes.size() - holenum_before_comp) * symm_group_num ;
    }

    double total_score = (double)total_beamhole_num / (double)total_coverd_holes_num;
//    printf("Total_beamhole: %d, Covered hole: %d , Total_Simp: %lf \n",
//           total_beamhole_num, total_coverd_holes_num, total_score);

    assert(total_score>=0.999 && total_score < 9999.0);

    return total_score;
}

double Grader::simplicity_online_3D(ConstructionGraph& constructionGraph){
    //TODO: havent test yet
    int total_beamhole_num = 0;
    int total_coverholes   = 0;
    for(auto& comp : constructionGraph.components){
        int symm_group_num = (int)comp->layerGraph->sketchGroup->getAllSymmetryGroups().size();
        total_beamhole_num += comp->gradeRecorder.simp_old_total_beamhole_num * symm_group_num ;
        total_coverholes   += comp->gradeRecorder.beamNumOnHole.size() * symm_group_num ;
    }
    assert(total_coverholes != 0);
    double simplicity_score_3D = (double)total_beamhole_num/total_coverholes;
    return simplicity_score_3D;
}


double Grader::rigidity_transitive(BeamLayout & beamLayout){

    //calculating total number of joint pairs
    int joint_pair_num = 0;

    //// Collect all ***covered*** edges
    set<UnitEdge*> all_edges;
    for(auto& b : beamLayout.beams){
        all_edges.insert(b->template_beam->covered_edges.begin(), b->template_beam->covered_edges.end());
    }

    for(auto& ue : all_edges){
        joint_pair_num += ue->getAdjacentEdges(all_edges).size();
    }
    joint_pair_num /= 2;


    int rigid_component_num = 0;

    set<BeamInstance*> all_beams(beamLayout.beams.begin(), beamLayout.beams.end());
    while( !all_beams.empty() ){
        BeamInstance* beamInstance = *all_beams.begin();
        set<BeamInstance*> rigid_component = beamLayout.getAllRigidConnectedBeams(beamInstance);
        for(auto& b : rigid_component)
            all_beams.erase(b);
        rigid_component_num++;
    }

    return (double)rigid_component_num/joint_pair_num;
}

double Grader::rigidity_transitive_online(BeamLayout & beamLayout, GradeRecorder& gradeRecorder) {

    return (double)gradeRecorder.rigid_beam_groups.size() / gradeRecorder.upairNum;

}

double Grader::rigidity_transitive_online_3D(ConstructionGraph& constructionGraph){
    int total_rigid_beam_groups = 0;
    int total_upair_num         = 0;
    for(auto& comp : constructionGraph.components){
        int symm_group_num = (int)comp->layerGraph->sketchGroup->getAllSymmetryGroups().size(); //number of symmetry groups

        total_rigid_beam_groups += comp->gradeRecorder.rigid_beam_groups.size() * symm_group_num ;
        total_upair_num         += comp->gradeRecorder.upairNum                 * symm_group_num ; //TODO: there may be 0 pairs under single hole condition
    }

    assert(total_upair_num != 0);
    return (double)total_rigid_beam_groups/total_upair_num;
}

double Grader::rigidity_transitive_3D(ConstructionGraph& constructionGraph){

    /////calculating total number of joint pairs

    int total_joint_pair_num = 0;
    for(auto& comp : constructionGraph.components){

        int comp_joint_pair_num = 0;

        //// Collect all ***covered*** edges
        set<UnitEdge*> all_edges;
        for(auto& b : comp->beamLayout->beams){
            all_edges.insert(b->template_beam->covered_edges.begin(), b->template_beam->covered_edges.end());
        }

        for(auto& ue : all_edges){
            comp_joint_pair_num += ue->getAdjacentEdges(all_edges).size();
        }
        comp_joint_pair_num /= 2;

        int symm_group_num = (int)comp->layerGraph->sketchGroup->getAllSymmetryGroups().size(); //number of symmetry groups
        total_joint_pair_num += comp_joint_pair_num * symm_group_num ;
    }

    int total_rigid_component_num = 0;
    for(auto& comp : constructionGraph.components){
        int comp_rigid_component_num = 0;

        set<BeamInstance*> all_beams(comp->beamLayout->beams.begin(),
                                     comp->beamLayout->beams.end());
        while( !all_beams.empty() ){
            BeamInstance* beamInstance = *all_beams.begin();
            set<BeamInstance*> rigid_component = comp->beamLayout->getAllRigidConnectedBeams(beamInstance);
            for(auto& b : rigid_component)
                all_beams.erase(b);
            comp_rigid_component_num++;
        }

        int symm_group_num = (int)comp->layerGraph->sketchGroup->getAllSymmetryGroups().size(); //number of symmetry groups
        total_rigid_component_num += comp_rigid_component_num * symm_group_num ;
    }


    return (double)total_rigid_component_num / total_joint_pair_num;

}


double Grader::gaps(BeamLayout& beamLayout){
    int gap_num = 0;
    set<LayerHole*> covered_holes;
    for(auto& b : beamLayout.beams){
        if(b->layer_number == NO_LAYER) continue;
        covered_holes.insert(b->template_beam->covered_holes.begin(),b->template_beam->covered_holes.end());
    }

    for(auto& h : covered_holes){
        gap_num += beamLayout.gapNumber(h);
    }
    return (double)gap_num/covered_holes.size();
}

double Grader::gaps_online(BeamLayout & beamLayout, GradeRecorder& gradeRecorder) {

    return (double)gradeRecorder.gap_number / gradeRecorder.layerNumOnHole.size();

}

double Grader::gaps_online_3D(ConstructionGraph& constructionGraph){
    int total_gap_num  = 0;
    int total_hole_num = 0;

    for(auto& comp : constructionGraph.components){
        int symm_group_num = (int)comp->layerGraph->sketchGroup->getAllSymmetryGroups().size(); //number of symmetry groups

        total_gap_num  += comp->gradeRecorder.gap_number            * symm_group_num ;
        total_hole_num += comp->gradeRecorder.layerNumOnHole.size() * symm_group_num ;
    }
    assert(total_hole_num != 0);

    return (double)total_gap_num / total_hole_num;
}

double Grader::gaps_3D(ConstructionGraph& constructionGraph){

    int total_gap_num  = 0;
    int total_hole_num = 0;

    for(auto& comp : constructionGraph.components){
        int gap_num = 0;
        set<LayerHole*> covered_holes;
        for(auto& b : comp->beamLayout->beams){
            if(b->layer_number == NO_LAYER) continue;
            covered_holes.insert(b->template_beam->covered_holes.begin(), b->template_beam->covered_holes.end());
        }

        for(auto& h : covered_holes){
            gap_num += comp->beamLayout->gapNumber(h);
        }

        int symm_group_num = (int)comp->layerGraph->sketchGroup->getAllSymmetryGroups().size(); //number of symmetry groups

        total_gap_num  += gap_num * symm_group_num ;
        total_hole_num += covered_holes.size() * symm_group_num ;
    }

    return (double)total_gap_num / total_hole_num;

}

double Grader::layer_bound(BeamLayout& beamLayout){
    int max_layer = -999999;
    int min_layer = 999999;
    for(auto& b : beamLayout.beams){
        if(b->layer_number == NO_LAYER) continue;
        if(b->layer_number > max_layer)   max_layer = b->layer_number;
        if(b->layer_number < min_layer)   min_layer = b->layer_number;
    }
    return (double)(max_layer-min_layer + 1);//加一是因为有两层beam的时候要使这个数字为2
}

double Grader::layer_bound_online(BeamLayout & beamLayout, GradeRecorder& gradeRecorder) {

    if( gradeRecorder.beamsSortedByLayer.empty() ){ //maybe empty set due to the removal operation
        return 0.0;
    }else{
        return  (*gradeRecorder.beamsSortedByLayer.rbegin())->layer_number -
                (*gradeRecorder.beamsSortedByLayer.begin()) ->layer_number + 1;
    }
}

//Pick the max bound among all components
double Grader::layer_bound_online_3D(ConstructionGraph& constructionGraph){

    double max_bound = 0;
    for(auto& comp : constructionGraph.components){
        double curr_bound = Grader::layer_bound_online(*comp->beamLayout, comp->gradeRecorder);
        if(curr_bound > 100){
            for(auto& b : comp->gradeRecorder.beamsSortedByLayer){
                printf("%d ", b->layer_number);
            }
            printf("\n");
        }
        assert(curr_bound < 100);
        max_bound = curr_bound > max_bound ? curr_bound : max_bound;
    }
    return max_bound;
}

double Grader::layer_bound_3D(ConstructionGraph& constructionGraph){
    double max_bound = 0;

    for(auto& comp : constructionGraph.components){
        double curr_bound = Grader::layer_bound(*comp->beamLayout);
        max_bound = curr_bound > max_bound ? curr_bound : max_bound;
    }

    return max_bound;
}

double Grader::layer_symmetry(BeamLayout& beamLayout)//The symmetry beam pair should be ideally have same layer number, this function evaluate this
{
    int layer_variation = 0;
    for(auto& b : beamLayout.beams){
        if(b->symm_beam != NULL){
            layer_variation += (b->layer_number - b->symm_beam->layer_number) *
                               (b->layer_number - b->symm_beam->layer_number);
        }
    }
    return (double)layer_variation / beamLayout.beams.size();
}

double Grader::layer_symmetry_online(BeamLayout & beamLayout, GradeRecorder& gradeRecorder) {
    return (double)gradeRecorder.symm_layer_deviation / beamLayout.beams.size();
}

double Grader::layer_symmetry_online_3D(ConstructionGraph& constructionGraph){

    int total_symm_deviation = 0;
    int total_beam_num       = 0;
    for(auto& comp : constructionGraph.components){
        int symm_group_num = (int)comp->layerGraph->sketchGroup->getAllSymmetryGroups().size(); //number of symmetry groups

        total_symm_deviation += comp->gradeRecorder.symm_layer_deviation * symm_group_num;
        total_beam_num       += comp->beamLayout->beams.size() * symm_group_num;
    }
    assert(total_beam_num != 0);

    return (double)total_symm_deviation / total_beam_num;
}

double Grader::layer_symmetry_3D(ConstructionGraph& constructionGraph){
    int total_symm_deviation = 0;
    int total_beam_num       = 0;
    for(auto& comp : constructionGraph.components){
        int layer_variation = 0;

        for(auto& b : comp->beamLayout->beams){
            if(b->symm_beam != NULL){
                layer_variation += (b->layer_number - b->symm_beam->layer_number) *
                                   (b->layer_number - b->symm_beam->layer_number);
            }
        }

        int symm_group_num = (int)comp->layerGraph->sketchGroup->getAllSymmetryGroups().size(); //number of symmetry groups

        total_symm_deviation += layer_variation                * symm_group_num ;
        total_beam_num       += comp->beamLayout->beams.size() * symm_group_num ;
    }

    return (double)total_symm_deviation / total_beam_num;

}

double Grader::completeness(BeamLayout& beamLayout){
 //TODO: havent consider single hole here
    set<UnitEdge*> covered_edges;
    for(auto& b : beamLayout.beams){
        covered_edges.insert(b->template_beam->covered_edges.begin(), b->template_beam->covered_edges.end());
    }

    return (double)covered_edges.size() / beamLayout.unit_edges.size();

}

double Grader::completeness_online(BeamLayout & beamLayout, GradeRecorder& gradeRecorder) {

    return (double)gradeRecorder.curr_edges.size() / beamLayout.unit_edges.size();

}

double Grader::completeness_online_3D(ConstructionGraph &constructionGraph){

    int total_covered_edges = 0;
    int total_unit_edges    = 0;
    for(auto& comp : constructionGraph.components){
        int symm_group_num = (int)comp->layerGraph->sketchGroup->getAllSymmetryGroups().size(); //number of symmetry groups

        total_covered_edges += comp->gradeRecorder.curr_edges.size() * symm_group_num;
        total_unit_edges    += comp->beamLayout->unit_edges.size()   * symm_group_num;
    }

    assert(total_unit_edges != 0);

    return (double) total_covered_edges / total_unit_edges;
}

double Grader::completeness_online_3D_inferenced(ConstructionGraph & constructionGraph) //put other components layer shifting to current component into consideration
{
    int total_covered_edges = 0;
    int total_unit_edges    = 0;
    for(auto& comp : constructionGraph.components){

        int curr_comp_covered_edges = 0;

        for( auto& ue : comp->beamLayout->unit_edges ){
            if(comp->gradeRecorder.curr_edges.find(ue) != comp->gradeRecorder.curr_edges.end() ||
                [ue, comp]() -> bool {
                    for(auto& adj_comp : comp->adj_components){
                        for(auto& b : adj_comp->beamLayout->beams){
                            auto beam = b->getBeam();
                            auto holes = beam->getCurrentComponentHoles();
                            for(auto& h : holes)
                                if( (ue->hole1->position()-h).norm() < 0.9 || (ue->hole2->position()-h).norm() < 0.9 )
                                    return true;
                                else if( ue->symm_uedge != NULL && ((ue->symm_uedge->hole1->position()-h).norm() < 0.9 || (ue->symm_uedge->hole2->position()-h).norm() < 0.9) )
                                     return true;
                            delete beam;
                        }
                    }
                    return false;
                }() == true
              )
            {
                curr_comp_covered_edges ++;
            }
        }

        int symm_group_num = (int)comp->layerGraph->sketchGroup->getAllSymmetryGroups().size(); //number of symmetry groups

        total_covered_edges += curr_comp_covered_edges             * symm_group_num;
        total_unit_edges    += comp->beamLayout->unit_edges.size() * symm_group_num;

    }

    assert(total_unit_edges != 0);

    return (double) total_covered_edges / total_unit_edges;
}

double Grader::completeness_3D(ConstructionGraph & constructionGraph){
    int total_coverd_edges = 0;
    int total_unit_edges   = 0;

    for(auto& comp : constructionGraph.components){

        set<UnitEdge*> covered_edges;
        for(auto& b : comp->beamLayout->beams){
            covered_edges.insert(b->template_beam->covered_edges.begin(), b->template_beam->covered_edges.end());
        }

        int symm_group_num = (int)comp->layerGraph->sketchGroup->getAllSymmetryGroups().size(); //number of symmetry groups

        total_coverd_edges += covered_edges.size() * symm_group_num ;
        total_unit_edges   += comp->beamLayout->unit_edges.size() * symm_group_num;
    }

    return (double)total_coverd_edges / total_unit_edges;

}

// collision = (number of collision holes) / (number of total holes)
double Grader::collision_3D(ConstructionGraph &constructionGraph){

    int total_beamhole_num_collid = 0;
    int total_beamhole_num        = 0;

    vector< tuple<Vector3d, int> > total_holes; //list of mapping from point to the number of same-position points


    for(auto& comp : constructionGraph.components){

        ////Collect all beams
        vector<BeamTemplate *> m_beams;
        for(auto& b : comp->beamLayout->beams) m_beams.push_back(b->getBeam());

        if(comp->layerGraph->containSymmetryComponentsAllDirection()){
            vector<BeamTemplate *> symm_beams = Symmetry::getSymmetryComponentsAsBeams(
                    m_beams, comp->layerGraph->getCurrentPlaneFlag(),
                    comp->layerGraph->sketchGroup->getAllSymmetryGroupsWithDirecTag());
            for(auto& b : symm_beams) b->color = 4;
            m_beams.insert(m_beams.end(),symm_beams.begin(),symm_beams.end());

        }else if(comp->layerGraph->sketchGroup->isSelfSymmetry()){
            //TODO: implement self symmetry here
        }

        ////calculate collision point
        for(auto& b : m_beams){
            auto beam_holes = b->getCurrentComponentHoles();
            for(auto& hole : beam_holes){
                ///// search for all existing holes
                auto find_iter = std::find_if(total_holes.begin(), total_holes.end(), [hole](tuple<Vector3d, int>& exist_hole){
                    return  (hole-std::get<0>(exist_hole)).norm() < 0.9; //collision if distance between two hole center is less than 1.0
                });

                if(find_iter == total_holes.end()){ // not exist yet
                    total_holes.push_back( std::make_tuple(hole, 1) );
                } else{ //already exist
                    std::get<1>(*find_iter) += 1;
                }
                total_beamhole_num ++;
            }
        }
    }

    total_beamhole_num_collid = (int)std::count_if(total_holes.begin(), total_holes.end(), [](tuple<Vector3d, int>& exist_hole){
                                                                                            return std::get<1>(exist_hole) > 1; });

    assert(total_beamhole_num != 0);
    return (double)total_beamhole_num_collid / total_beamhole_num;
}

double Grader::collision_num_3D(ConstructionGraph & constructionGraph) // return the number of collison holes
{
    int total_beamhole_num_collid = 0;
    int total_beamhole_num        = 0;

    vector< tuple<Vector3d, int> > total_holes; //list of mapping from point to the number of same-position points


    for(auto& comp : constructionGraph.components){

        ////Collect all beams
        vector<BeamTemplate *> m_beams;
        for(auto& b : comp->beamLayout->beams) m_beams.push_back(b->getBeam());

        if(comp->layerGraph->containSymmetryComponentsAllDirection()){
            vector<BeamTemplate *> symm_beams = Symmetry::getSymmetryComponentsAsBeams(
                    m_beams, comp->layerGraph->getCurrentPlaneFlag(),
                    comp->layerGraph->sketchGroup->getAllSymmetryGroupsWithDirecTag());
            for(auto& b : symm_beams) b->color = 4;
            m_beams.insert(m_beams.end(),symm_beams.begin(),symm_beams.end());

        }else if(comp->layerGraph->sketchGroup->isSelfSymmetry()){
            //TODO: implement self symmetry here
        }

        ////calculate collision point
        for(auto& b : m_beams){
            auto beam_holes = b->getCurrentComponentHoles();
            for(auto& hole : beam_holes){
                ///// search for all existing holes
                auto find_iter = std::find_if(total_holes.begin(), total_holes.end(), [hole](tuple<Vector3d, int>& exist_hole){
                    return  (hole-std::get<0>(exist_hole)).norm() < 0.9; //collision if distance between two hole center is less than 1.0
                });

                if(find_iter == total_holes.end()){ // not exist yet
                    total_holes.push_back( std::make_tuple(hole, 1) );
                } else{ //already exist
                    std::get<1>(*find_iter) += 1;
                }
                total_beamhole_num ++;
            }
        }
    }

    total_beamhole_num_collid = (int)std::count_if(total_holes.begin(), total_holes.end(), [](tuple<Vector3d, int>& exist_hole){
        return std::get<1>(exist_hole) > 1; });

    assert(total_beamhole_num != 0);
    return (double)total_beamhole_num_collid;
}

double Grader::overall_score_2D(BeamLayout& beamLayout, bool print_flag, string &str){
    /* extremly simplicity settings */
    double w1 = 0.0001, w2 = 100, w3 = 0.0, w4 = 500, w5 = 0.03, w6 = 0.05;
    /* extremly similar to sketch settings */
//    double w1 = 200, w2 = 10, w3 = 0.0, w4 = 500, w5 = 0.03, w6 = 0.05;

    double deviation_score  = deviation2sketch(beamLayout);
    double simplicity_score = simplicity(beamLayout);
    double rigidity_score   = rigidity_transitive(beamLayout);
    double gaps_score       = gaps(beamLayout);
    double layerbound_score = layer_bound(beamLayout);
    double layer_symm       = layer_symmetry(beamLayout);
    double total_score      = w1 * deviation_score +
                              w2 * simplicity_score +
                              w3 * rigidity_score +
                              w4 * gaps_score +
                              w5 * layer_bound(beamLayout)*layer_bound(beamLayout) +
                              w6 * layer_symm;

    if(print_flag){
        /* record the result quality to string */
        char buff[200];
        snprintf(buff, sizeof(buff), "deviation: %lf\t, simplicity: %lf\t, rigidity:  %lf\t, gap:  %lf\t, bound: %lf\t, symmetry: %lf\t, total: %lf\n",
                 deviation_score, simplicity_score, rigidity_score, gaps_score, layerbound_score, layer_symm, total_score);
        str += buff;
        char buff_weight[200];
        snprintf(buff_weight, sizeof(buff_weight), "w1: %lf\t, w2: %lf\t, w3:  %lf\t, w4: %lf\t, w5: %lf\t, w6: %lf\t, total_score: %lf\n",
                 w1,w2,w3,w4,w5,w6,total_score);
        str += buff_weight;

        printf("deviation: %lf\t, simplicity: %lf\t, rigidity:  %lf\t, gap:  %lf\t, bound: %lf\t, symmetry: %lf\t, total: %lf\n",
               deviation_score, simplicity_score, rigidity_score, gaps_score, layerbound_score, layer_symm, total_score);

        printf("deviation: %lf\t, simplicity: %lf\t, rigidity:  %lf\t, gap:  %lf\t, bound: %lf\t, layer_symmetry: %lf\t, total: %lf\n",
               w1 * deviation_score, w2 * simplicity_score,
               w3 * rigidity_score,  w4 * gaps_score,
               w5 * pow(w5,layer_bound(beamLayout)) , w6 * layer_symm, total_score);
    }


    return total_score;
}

double Grader::overall_score_2D_online(BeamLayout & beamLayout, GradeRecorder & gradeRecorder,
                                       bool print_flag, string &str){
    /* extremly simplicity settings */
    double w1 = 0.0001, w2 = 100, w3 = 0.0, w4 = 500, w5 = 0.03, w6 = 0.05;
    /* extremly similar to sketch settings */
//    double w1 = 200, w2 = 10, w3 = 0.0, w4 = 500, w5 = 0.03, w6 = 0.05;

    double deviation_score  = deviation2sketch_online(beamLayout, gradeRecorder);
    double simplicity_score = simplicity_online(beamLayout, gradeRecorder);
    double rigidity_score   = rigidity_transitive_online(beamLayout, gradeRecorder);
    double gaps_score       = gaps_online(beamLayout, gradeRecorder);
    double layerbound_score = layer_bound_online(beamLayout, gradeRecorder);
    double layer_symm       = layer_symmetry_online(beamLayout, gradeRecorder);
    double completenes      = completeness_online(beamLayout, gradeRecorder);
    double total_score      = w1 * deviation_score +
                              w2 * simplicity_score +
                              w3 * rigidity_score +
                              w4 * gaps_score +
                              w5 * layer_bound(beamLayout)*layer_bound(beamLayout) +
                              w6 * layer_symm;

    if(print_flag){
        /* record the result quality to string */
        char buff[200];
        snprintf(buff, sizeof(buff), "deviation: %lf\t, simplicity: %lf\t, rigidity:  %lf\t, gap:  %lf\t, bound: %lf\t, symmetry: %lf\t, total: %lf\n",
                 deviation_score, simplicity_score, rigidity_score, gaps_score, layerbound_score, layer_symm, total_score);
        str += buff;
        char buff_weight[200];
        snprintf(buff_weight, sizeof(buff_weight), "w1: %lf\t, w2: %lf\t, w3:  %lf\t, w4: %lf\t, w5: %lf\t, w6: %lf\t, total_score: %lf\n",
                 w1,w2,w3,w4,w5,w6,total_score);
        str += buff_weight;

        printf("deviation: %lf\t, simplicity: %lf\t, rigidity:  %lf\t, gap:  %lf\t, bound: %lf\t, symmetry: %lf\t, total: %lf\n",
               deviation_score, simplicity_score, rigidity_score, gaps_score, layerbound_score, layer_symm, total_score);

        printf("deviation: %lf\t, simplicity: %lf\t, rigidity:  %lf\t, gap:  %lf\t, bound: %lf\t, layer_symmetry: %lf\t, total: %lf\n",
               w1 * deviation_score, w2 * simplicity_score,
               w3 * rigidity_score,  w4 * gaps_score,
               w5 * pow(w5,layer_bound(beamLayout)) , w6 * layer_symm, total_score);
    }


    return total_score;
}

double Grader::overall_score_3D(ConstructionGraph &constructionGraph, bool consider_connection ,bool consider_collision, bool print_flag){
    /* extremly simplicity settings */
    auto weight = glob_vars::current_model.generating_weight;
    ////// dev                simp                 rigid
    double w_dev = weight[0], w_simp = weight[1], w_rigid = weight[2] ;
    ////// gap                bound                symm                  collision            connection
    double w_gap = weight[3], w_bound = weight[4], w_lsymm = weight[5],  w_colld = weight[6], w_conn = weight[7] ;

    double deviation_score    = 0.0;
    double simplicity_score   = 0.0;
    double rigidity_score     = 0.0;
    double gaps_score         = 0.0;
    double layerbound_score   = 0.0;
    double layer_symm         = 0.0;
    double collision_score    = 0.0; //TODO: evaluate collision 的时候symmetry 也有问题
    double connection_score   = 0.0;

    /**** Deviation ****/
    deviation_score           = Grader::deviation2sketch_online_3D(constructionGraph);
//    double dev_validate = Grader::deviation2sketch_3D(constructionGraph);
//    printf("deviation:(full, local) (%lf, %lf)\n", dev_validate, deviation_score);
//    assert(dev_validate == deviation_score);

    /**** Simplicity ****/
    simplicity_score          = Grader::simplicity_online_3D(constructionGraph);
//    double simp_validate = Grader::simplicity_3D(constructionGraph);
//    printf("simplicity:(full, local) (%lf, %lf)\n", simp_validate, simplicity_score);
//    assert(simp_validate == simplicity_score);

    /**** Rigidity ****/
    rigidity_score             = Grader::rigidity_transitive_online_3D(constructionGraph);
//    double rigidity_validate = Grader::rigidity_transitive_3D(constructionGraph);
//    printf("rigidity: (full, local) (%lf, %lf)\n", rigidity_validate, rigidity_score);
//    assert(rigidity_validate == rigidity_score );

    /**** Gaps ****/
    gaps_score                 = Grader::gaps_online_3D(constructionGraph);
//    double gap_validate = Grader::gaps_3D(constructionGraph);
//    printf("gaps: (full, local) (%lf, %lf)\n", gap_validate, gaps_score);
//    assert(gap_validate == gaps_score);

    /**** Layer Bound ****/
    layerbound_score           = Grader::layer_bound_online_3D(constructionGraph);
//    double layerbound_validate = Grader::layer_bound_3D(constructionGraph);
//    printf("layerbound: (full, local) (%lf, %lf)\n", layerbound_validate, layerbound_score);
//    assert(layerbound_validate == layerbound_score);

    /**** Layer Symmetry ****/
    layer_symm                 = Grader::layer_symmetry_online_3D(constructionGraph);
//    double layer_symm_validate = Grader::layer_symmetry_3D(constructionGraph);
//    printf("layer_symm: (full, local) (%lf, %lf)\n", layer_symm_validate, layer_symm);
//    assert(layer_symm_validate == layer_symm);

    /**** Collision ****/
//    collision_score = (double)constructionGraph.beamLayoutConnectorGenerator->getCollisionNum() / glob_vars::sketchGraph->m_holes.size();
    collision_score  = consider_collision ?
                       Grader::collision_3D(constructionGraph) : 0.0 ;
//    printf("collison:%lf\n", collision_score);

    static double record_connection_num = 0;
    connection_score = consider_connection ?
                       Grader::connection_score_3D(constructionGraph) : record_connection_num;
    record_connection_num = connection_score;


    double total_score      = w_dev   * deviation_score +
                              w_simp  * simplicity_score +
                              w_rigid * rigidity_score +
                              w_gap   * gaps_score +
                              w_bound * layerbound_score * layerbound_score +
                              w_lsymm * layer_symm +
                              w_colld * collision_score +
                              w_conn  * connection_score ;

    if(print_flag)
        printf("deviation: %lf\t, simplicity: %lf\t, rigidity:  %lf\t, gap:  %lf\t, bound: %lf\t,     symmetry: %lf\t, collision: %lf\t, connection: %lf    total: %lf\n",
               deviation_score,   simplicity_score,  rigidity_score,   gaps_score,  layerbound_score, layer_symm,      collision_score,  connection_score , total_score);

    return total_score;
}


double Grader::connection_score_3D(ConstructionGraph & constructionGraph){

    BeamLayoutConnectorGenerator connectorGenerator( &constructionGraph );
    int unCoveredEdges = connectorGenerator.genConnectorsCoverUnCoveredEdges().size();
    return unCoveredEdges;

}

double Grader::Nonpin_head_ratio(const vector<Connector *> &connectors){
    if(connectors.empty()){
        printf("The connectors are empty!!\n");
        cin.get();
    }
    int inst_pin_num = 0;
    int non_inst_pin_num = 0;
    for(const auto & c : connectors){
        inst_pin_num     += c->pinNum();
        non_inst_pin_num += c->non_pinNum();
    }
    return (double)non_inst_pin_num/(double)(inst_pin_num+non_inst_pin_num);
}


double Grader::pinhead_ratio(const vector<Connector> &connectors){
    int inst_pin_num = 0;
    int non_inst_pin_num = 0;
    for(const auto & c : connectors){
        inst_pin_num     += c.pinNum();
        non_inst_pin_num += c.non_pinNum();
    }
    return (double)inst_pin_num/(double)(inst_pin_num+non_inst_pin_num);
}
//the number of pin heads
int Grader::pin_num(const vector<Connector*>& connectors){
    int result = 0;
    for(auto& c : connectors){
        result += c->pinNum();
    }
    return result;
}

double Grader::sketchOrthorgNum_enhenceNonOrthorg(const SketchGraph* sketchGraph){
    double result = 0;
    for(auto& p : *sketchGraph->getPoints()){
        result += p->scoreNonOrthorNum_enhenceNonOrthorg();
    }
    return result;
}