//
// Created by 徐豪 on 2017/7/4.
//

#include "SketchGraph.h"
#include "fstream"
#include "util/util.h"
#include <iostream>
#include "configure/global_variables.h"
#include "SketchGroup.h"
#include <map>
#include "LayerGraph.h"
#include "objectives/Grader.h"
#include "IO/DataWriter.h"
#include <ctime>
#include "configure/Constants.h"


using std::cout;
using std::endl;

void SketchGraph::estabilishAdjacentGroup()
{
    set<SketchGroup*> all_groups;
    for(auto & g : this->group_xz){
        all_groups.insert(g);
    }
    for(auto & g: this->group_xy){
        all_groups.insert(g);
    }
    for(auto & g : this->group_yz){
        all_groups.insert(g);
    }
    for(auto & g : this->uncertain_group){
        all_groups.insert(g);
    }

    for(auto iter = all_groups.begin(); iter != all_groups.end() ; iter++){
        for(auto iter2 = all_groups.begin(); iter2 != all_groups.end() ; iter2++){
            if((*iter) != (*iter2) && (*iter)->connect(*iter2)){
                (*iter)->adjacentGroups.insert(*iter2);
            }
        }
    }
}

void SketchGraph::writeObj(std::string path)
{
    using std::ofstream;
    ofstream fout;
    fout.open(path);
    fout<<"# Auto Generated by Program \n";
    for(int i = 0;i<m_points->size();i++)
    {
        SketchPoint* p = m_points->at(i);
        fout<<"v "<< p->_x<<" "<<p->_y<<" "<<p->_z<<"\n";
    }
    fout<<"g Spine\n";
    for(int i = 0;i<m_lines->size();i++)
    {
        SketchPoint *p1 = m_lines->at(i)->_p1;
        SketchPoint *p2 = m_lines->at(i)->_p2;
        fout<<"g Segment"<<i<<"\n";
        fout<<"l "<<p1->_id+1<<" "<<p2->_id+1<<"\n"; //here +1 because the index in obj file start from 1, but our program start from 0
    }
    std::cout<<"File \""<<path<<"\" generated\n";
    fout.close();
}


void SketchGraph::estabilshSketchSymmetry(){
    for(auto& l1 : *m_lines){
        for(auto& l2 : *m_lines){
            if(l1 != l2){
                if(glob_vars::current_model.symmetry_x && l1->isAxisSymmetry(l2, Constants::PLANE_YZ)){
                    l1->ref_symm_line_x = l2;
                }
                if(glob_vars::current_model.symmetry_y && l1->isAxisSymmetry(l2, Constants::PLANE_XZ)){
                    l1->ref_symm_line_y = l2;
                }
                if(glob_vars::current_model.symmetry_z && l1->isAxisSymmetry(l2, Constants::PLANE_XY)){
                    l1->ref_symm_line_z = l2;
                }
            }
        }
    }
}

bool sortPointsX(SketchPoint* pi, SketchPoint* pj){ return pi->_x < pj->_x;}

vector<SketchGroup*> extractConnectorComponents(set<SketchLine*>& lines, const vector<SketchGroup*>& group_sequence){
    vector<SketchGroup*> connectorComponents;
    set<SketchLine*> buffered_lines;
    while(!lines.empty()){
        SketchGroup* group = new SketchGroup;
        auto line = *lines.begin();
        auto adj_lines = line->getAllAdjLines(lines);
        assert(isStraightLines(adj_lines));
        group->lines.insert(adj_lines.begin(),adj_lines.end());
        group->orientation_tag = line->possi_orientation;
        connectorComponents.push_back(group);
        buffered_lines.insert(adj_lines.begin(),adj_lines.end());
        for(auto& l : adj_lines)
            lines.erase(l);
    }
    SketchGraph::establishSymmetryGroupRelation(connectorComponents);
    lines.insert(buffered_lines.begin(),buffered_lines.end());
    return connectorComponents;
}

void SketchGraph::findPossibleDecompose(set<SketchLine *> &lines, int k, vector<SketchGroup *> &group_sequence,
                                        vector<vector<SketchGroup *>> &results){
    vector<SketchGroup*> curr_result;
    findKLargestComponents(lines, k, curr_result);
    if(curr_result.empty()){
        auto connComponents = extractConnectorComponents(lines,group_sequence);
        vector<SketchGroup*> curr_group = group_sequence;
        curr_group.insert(curr_group.end(),connComponents.begin(),connComponents.end());
        results.push_back(curr_group);
    } else{
        for(auto& g : curr_result){
            group_sequence.push_back(g);
            for(auto& l : g->lines)
                lines.erase(l);
            if(!g->getAllSymmetryGroups().empty()){
                for(auto& sg : g->getAllSymmetryGroups()){
                    group_sequence.push_back(sg);
                    for(auto& l : sg->lines)
                        lines.erase(l);
                }
            }
            findPossibleDecompose(lines, k, group_sequence, results);
            group_sequence.pop_back();
            lines.insert(g->lines.begin(), g->lines.end());
            if(!g->getAllSymmetryGroups().empty()){
                for(auto& sg : g->getAllSymmetryGroups()){
                    group_sequence.pop_back();
                    lines.insert(sg->lines.begin(), sg->lines.end());
                }
            }
        }
    }
}

void SketchGraph::initLinesOrientation()//使用greedy的sweep plane来对每一条线段赋予方向
{
    vector<vector<SketchGroup*>> init_state;
    set<SketchLine*> lines((*this->m_lines).begin(),(*this->m_lines).end());
    vector<SketchGroup*> group_sequence;
    findPossibleDecompose(lines, 1, group_sequence, init_state);
    assert(init_state.size() == 1);
    for(auto& g : init_state.back()){
        g->assignOrientInfo();
    }
}

void SketchGraph::randomInitLinesOrientation()
{
    for(auto& l : *this->m_lines){
        int l_chosen = intRandom(0,l->possi_orient_set.size()-1);
        l->chosen_orientation = l->possi_orient_set.at(l_chosen);
        if(l->ref_symm_line_x != NULL){
            l->ref_symm_line_x->chosen_orientation = l->possi_orient_set.at(l_chosen);
        }
    }
}

void SketchGraph::initPointsOrientation()
{
    for(auto& p : *this->m_points){
        p->chooseMinObjfuncDirection();
    }
}

double SketchGraph::doRandomOperation_AllSymmetry(double& curr_score, SketchLine*& changed_line, int& prev_chosen){
    std::random_shuffle(m_points->begin(),m_points->end());
    for(auto& p : *m_points){
        auto adj_lines = p->getAdjLines();
        for(auto& l : adj_lines){
            if (l->chosen_orientation != p->chosen_orientation){
                std::random_shuffle(adj_lines.begin(),adj_lines.end());
                for(auto& dl : adj_lines){
                    if(dl->possi_orient_set.size() == 1)
                        continue;
                    prev_chosen = dl->chosen_orientation;
                    changed_line = dl;
                    int dl_chosen = dl->randomChangeOrientation();
                    dl->_p1->chooseMinObjfuncDirection();
                    dl->_p2->chooseMinObjfuncDirection();
                    set<SketchLine*> symm_lines = dl->getAllSymmetryLines();
                    for(auto& symm_l : symm_lines){
                        symm_l->chosen_orientation = dl_chosen;
                        symm_l->_p1->chooseMinObjfuncDirection();
                        symm_l->_p2->chooseMinObjfuncDirection();
                    }
                    double score = Grader::sketchOrthorgNum_enhenceNonOrthorg(this);
                    curr_score = score;
                    return curr_score;
                }
            }
        }
    }
    return 0.0;
}

void SketchGraph::undoOperationAllSymmetry(SketchLine *changed_line, int &prev_chosen){
    changed_line->chosen_orientation = prev_chosen;
    changed_line->_p1->chooseMinObjfuncDirection();
    changed_line->_p2->chooseMinObjfuncDirection();
    for(auto& symm_l : changed_line->getAllSymmetryLines()){
        symm_l->chosen_orientation = prev_chosen;
        symm_l->_p1->chooseMinObjfuncDirection();
        symm_l->_p2->chooseMinObjfuncDirection();
    }
}


void SketchGraph::decomposeBySA(){
    //start to find optimization solutioin..
    double curr_min_num = Grader::sketchOrthorgNum_enhenceNonOrthorg(this);
    double curr_score = curr_min_num;
    DataWriter::writeHoleOrientation(*m_lines, *m_points, std::to_string(curr_score) + "_init");

    //SA objective function statistics
    std::ostringstream sa_statisctis;

    double T = 2000.0;   //TODO: adjust the max temperature to be higher
    double T_min = 0.01; //TODO: adjust the min temperature to be lower
    double r = 1 - ((double) 1 / ( 10*totalSketchLength() ));;

    glob_vars::config_file << "Max temperature: " << T     << endl;
    glob_vars::config_file << "Min temperature: " << T_min << endl;
    glob_vars::config_file << "Cooling rate   : " << r     << endl;
    glob_vars::config_file << "Initial score: " << curr_score<< endl;

    int iter = 0;
    while(T > T_min){
        SketchLine* changed_line = NULL;
        double prev_score = curr_score;
        int prev_operation = -1;
        double dt = curr_score - doRandomOperation_AllSymmetry(curr_score, changed_line, prev_operation);
        if(glob_vars::current_model.name == "fail_1")
            break;
        if(changed_line == NULL){
            cin.get();
        }
        if( iter % 500 == 0 ){
            printf("current score: %lf T: %lf \n", curr_score, T);
#ifdef OUTPUT_ANIMATION
            DataWriter::writeHoleOrientation(*m_lines, *m_points, "_annim_" + std::to_string(iter / 500));
#endif
        }
        if(dt>=0){//说明这个操作是下降的
            //accept this operation, do nothing
        }else{
            if(exp((dt)/T) > cRandom(0,1)){
                //accept this operation, do nothing
            }else{
                undoOperationAllSymmetry(changed_line, prev_operation);
                curr_score = prev_score;
            }
        }
        T = r * T ; //anealing
        if(curr_score == 0.00){
            printf("SA Already get 0, don't continue...\n");
            break;
        }
#ifdef OUTPUT_ANIMATION
        sa_statisctis << curr_score << "\n";
#endif
        iter++;
    }

//    if(glob_vars::current_model.name == "0_lifter"){
//        for(auto& l : *m_lines){
//            if(l->_id == 44 || l->_id == 45){
//                l->chosen_orientation = LayerGraph::PLANE_YZ;
//            }
//        }
//    }else if(glob_vars::current_model.name == "0_3Dprinter"){
//        for(auto& l : *m_lines){
//            if(l->_id == 53 || l->_id == 33){
//                l->chosen_orientation = LayerGraph::PLANE_YZ;
//                l->_p1->chooseMinObjfuncDirection();
//                l->_p2->chooseMinObjfuncDirection();
//            }
////            || l->_id == 159 || l->_id == 160
//            if(l->_id == 47 || l->_id == 40 ){
//                l->chosen_orientation = LayerGraph::PLANE_XY;
//                l->_p1->chooseMinObjfuncDirection();
//                l->_p2->chooseMinObjfuncDirection();
//            }
//        }
//    }else if(glob_vars::current_model.name == "0_tokyo_temple"){
//        /*** Top part of the tokyo temple ***/
////        for(auto& l : *m_lines){
////            if(l->_id == 89 || l->_id == 88 || l->_id == 73 || l->_id == 84){
////                l->chosen_orientation = LayerGraph::PLANE_YZ;
////                l->_p1->chooseMinObjfuncDirection();
////                l->_p2->chooseMinObjfuncDirection();
////            }
////        }
//        /*** Bottom part of the tokyo temple ***/
////        for(auto& l : *m_lines){
////            if(l->_id == 74 || l->_id == 73 || l->_id == 64 || l->_id == 70){
////                l->chosen_orientation = LayerGraph::PLANE_YZ;
////                l->_p1->chooseMinObjfuncDirection();
////                l->_p2->chooseMinObjfuncDirection();
////            }
////        }
//    }

//    printCurrentHoleOrientationStatus();
    glob_vars::config_file << "Final score: " << curr_score<< endl;
    DataWriter::writeHoleOrientation(*m_lines, *m_points, std::to_string(curr_score) + "_final");
    DataWriter::writeDecomposeSAStatistics(sa_statisctis.str(), "holeOrientation");
    printf("Final score:%lf:\n",curr_score);
    printf("SA end\n");
}

void SketchGraph::estimateLocalBeamOrientation(){

    initOrientationCandidates();
    sketchInitFreeTinyLines();
    randomInitLinesOrientation();
    initPointsOrientation();

    glob_vars::config_file << "------------- Orientation Estimation SA ----------------" << endl;
    clock_t start = clock();

    decomposeBySA();

    glob_vars::config_file << "Total Time: \t" << (double)(clock()-start)/CLOCKS_PER_SEC << endl;

}


void SketchGraph::establishSymmetryGroupRelation(vector<SketchGroup *> &groups){

    for(auto group1 = groups.begin(); group1 != groups.end(); group1++){
        for(auto group2 = group1 +1; group2 != groups.end(); group2++){
            set<int> symmDirections = (*group1)->symmetryDirections(*group2);
            if(!symmDirections.empty()){
//                if(!(*group1)->symmetryGroups.empty()){
//                    printf("more than one symmetry group!!!\n");
//                    for(auto& d :(*group1)->symmetryGroups) printf("%d\n",std::get<1>(d));
//                    cin.get();
//                }
                if(symmDirections.find(Constants::X_AXIS) != symmDirections.end() && glob_vars::current_model.symmetry_x){
                    (*group1)->symm_group_x = (*group2);
                    (*group2)->symm_group_x = (*group1);
                }
                if(symmDirections.find(Constants::Y_AXIS) != symmDirections.end() && glob_vars::current_model.symmetry_y){
                    (*group1)->symm_group_y = (*group2);
                    (*group2)->symm_group_y = (*group1);
                }
                if(symmDirections.find(Constants::Z_AXIS) != symmDirections.end() && glob_vars::current_model.symmetry_z){
                    (*group1)->symm_group_z = (*group2);
                    (*group2)->symm_group_z = (*group1);
                } else{
                    printf("这什么玩意\n");
                }
            }
        }
    }



}

void SketchGraph::sketchInitFreeTinyLines() const{
    //Let 1 and 2 length segments have 3 complete freedom
    for(auto& l : *m_lines){
        assert(l->intLength()>0);
        if(l->intLength() <= 2){
            if(glob_vars::current_model.name == "siggraph" || glob_vars::current_model.name == "fail_2") continue;
#ifdef CROSSBOW_HACKING
    if(l->_id == 19 || l->_id == 24 || l->_id == 25 || l->_id == 18 )
        continue;
#endif
            l->possi_orient_set.clear();
            l->possi_orient_set.push_back(Constants::PLANE_XY);
            l->possi_orient_set.push_back(Constants::PLANE_YZ);
            l->possi_orient_set.push_back(Constants::PLANE_XZ);
        }
    }
}

void SketchGraph::initOrientationCandidates() const {
    //Find main plane lines
    for(auto line : *m_lines)
    {
        assert(line->_p1 != line->_p2);

        double real_len = line->getVector().norm();

        Vector3d vec_xz = line->getVector(); vec_xz.y()=0;
        double proj_len_xz = vec_xz.norm();
        double diff_xz = fabs(real_len-proj_len_xz);

        Vector3d vec_xy = line->getVector(); vec_xy.z()=0;
        double proj_len_xy = vec_xy.norm();
        double diff_xy = fabs(real_len-proj_len_xy);

        Vector3d vec_yz = line->getVector(); vec_yz.x()=0;
        double proj_len_yz = vec_yz.norm();
        double diff_yz = fabs(real_len-proj_len_yz);

        if(diff_xz<0.05 && (diff_xy>0.05) && (diff_yz>0.05)){
            line->possi_orientation = Constants::PLANE_XZ;
            line->possi_orient_set.push_back(Constants::PLANE_XZ);
        }else if(diff_xz<0.05 && (diff_xy<0.05) && (diff_yz>0.05)){
            line->possi_orientation = Constants::PLANE_XY_OR_XZ;
            line->possi_orient_set.push_back(Constants::PLANE_XY);
            line->possi_orient_set.push_back(Constants::PLANE_XZ);
        }else if(diff_xz<0.05 && (diff_xy>0.05) && (diff_yz<0.05)){
            line->possi_orientation = Constants::PLANE_YZ_OR_XZ;
            line->possi_orient_set.push_back(Constants::PLANE_YZ);
            line->possi_orient_set.push_back(Constants::PLANE_XZ);
        }else if(diff_xz>0.05 && (diff_xy<0.05) && (diff_yz>0.05)){
            line->possi_orientation = Constants::PLANE_XY;
            line->possi_orient_set.push_back(Constants::PLANE_XY);
        }else if(diff_xz>0.05 && (diff_xy<0.05) && (diff_yz<0.05)){
            line->possi_orientation = Constants::PLANE_XY_OR_YZ;
            line->possi_orient_set.push_back(Constants::PLANE_XY);
            line->possi_orient_set.push_back(Constants::PLANE_YZ);
        }else if(diff_xz>0.05 && (diff_xy>0.05) && (diff_yz<0.05)){
            line->possi_orientation = Constants::PLANE_YZ;
            line->possi_orient_set.push_back(Constants::PLANE_YZ);
        }else {
            printf("Plane classification Error!!\n");
        }
    }
}


bool compareBySegmentLength(SketchGroup* g1, SketchGroup* g2){ return g1->totalSketchLength() > g2->totalSketchLength(); }
void SketchGraph::findKLargestComponents(set<SketchLine *> &lines, int k, vector<SketchGroup *> &all_groups){
    assert(all_groups.empty());
    sweepPlane(Constants::PLANE_XZ,lines,all_groups,false);
    sweepPlane(Constants::PLANE_XY,lines,all_groups,false);
    sweepPlane(Constants::PLANE_YZ,lines,all_groups,false);
    std::sort(all_groups.begin(),all_groups.end(),compareBySegmentLength);
    all_groups = vector<SketchGroup*>(all_groups.begin(),(k+1)>all_groups.size()? all_groups.end(): (all_groups.begin()+k+1));//有可能最后两个是symmetry的，所以这里考虑k+1个情况

    if(all_groups.size() > k){
        auto last_group = all_groups.back();
        bool last_group_symmetry = false;
        for(int i = 0; i < all_groups.size()-1; i++){//如果最后一个group与里面其中任意一个symmetry都不能把它删掉
            if(!all_groups.at(i)->symmetryDirections(last_group).empty()){
                last_group_symmetry = true;
                break;
            }
        }
        if(!last_group_symmetry)
            all_groups.pop_back();
    }

    establishSymmetryGroupRelation(all_groups);
    eraseSymmetryGroups(all_groups);//为保证symmetry group同时出现,虽然这里erase掉了，但后面会再放回去的
}

void SketchGraph::groupLinesByChosenDirection(set<SketchLine *> &lines, vector<SketchGroup *> &result_groups)
{
    sweepPlaneByChosenOrient(Constants::PLANE_XY, lines, result_groups, false);
    sweepPlaneByChosenOrient(Constants::PLANE_XZ, lines, result_groups, false);
    sweepPlaneByChosenOrient(Constants::PLANE_YZ, lines, result_groups, false);
}

int SketchGraph::totalSketchLength(){
    double length = 0.0;
    for(auto& l : *m_lines){
        length += l->length();
    }
    return  (int)round(length);
}

bool sortGroups(SketchGroup* gi, SketchGroup* gj){
    if((*gi->lines.begin())->_p1->_y < (*gj->lines.begin())->_p1->_y){
        return true;
    }else if((*gi->lines.begin())->_p1->_y >(*gj->lines.begin())->_p1->_y){
        return false;
    }else{//equal along y direction
        if((*gi->lines.begin())->_p1->_x < (*gj->lines.begin())->_p1->_x){
            return true;
        }else if((*gi->lines.begin())->_p1->_x > (*gj->lines.begin())->_p1->_x){
            return false;
        }else{//equal along x and y direction
            if((*gi->lines.begin())->_p1->_z < (*gj->lines.begin())->_p1->_z){
                return true;
            }else if((*gi->lines.begin())->_p1->_z > (*gj->lines.begin())->_p1->_z){
                return false;
            }else{//equal along x, y , and z direction
                return true;
            }
        }
    }
}

void SketchGraph::eraseSymmetryGroups(vector<SketchGroup*>& m_all_groups){
    set<SketchGroup*> real_groups_without_symm;//最后留下来的真正的group

    while( !m_all_groups.empty() ){
        SketchGroup* group = *m_all_groups.begin();
        vector<SketchGroup*> symm_group = group->getAllSymmetryGroups();

        //通过y,x,z方向依次排序，选择最靠右下的group留下，
        std::sort(symm_group.begin(), symm_group.end(), sortGroups);
        real_groups_without_symm.insert(symm_group.back());

        for(auto& g : symm_group)
            m_all_groups.erase(std::remove_if(m_all_groups.begin(),
                                              m_all_groups.end(),
                               [g](SketchGroup* group){return group == g;}), m_all_groups.end());
    }

    m_all_groups = vector<SketchGroup*>(real_groups_without_symm.begin(), real_groups_without_symm.end());
}

void SketchGraph::sweepPlane(int plane_flag, set<SketchLine *> &lines, vector<SketchGroup*> &result_group, bool erase){
    set<SketchLine*> buffer_lines;
    while(getGeneralizedDesiredLine(lines, plane_flag) != NULL){
        SketchGroup* group = new SketchGroup;
        auto line = getGeneralizedDesiredLine(lines, plane_flag);
        auto adj_lines = line->getAllAdjLinesByPossiOrient(lines, plane_flag);
        if(!isStraightLines(adj_lines)|| (isStraightLines(adj_lines) && isStraightNonOrthorgLine(adj_lines, plane_flag))){
            group->lines.insert(adj_lines.begin(),adj_lines.end());
            group->orientation_tag = plane_flag;
            result_group.push_back(group);
        }else{
            buffer_lines.insert(adj_lines.begin(),adj_lines.end());
        }
        for(auto& l : adj_lines)
            lines.erase(l);
    }
    lines.insert(buffer_lines.begin(), buffer_lines.end());
    if(!erase){
        for(auto& group : result_group){
            lines.insert(group->lines.begin(), group->lines.end());
        }
    }
}

void SketchGraph::sweepPlaneByChosenOrient(int orientation_tag, set<SketchLine *> &lines,
                              vector<SketchGroup*> &result_group, bool erase) //将lines按照 decomposition 算法之后的chosen 平面来decompose结果放在group中，并且根据erase变量来看是否需要erase已经group 的line
{

    while(getExactlyDesiredLine(lines, orientation_tag) != NULL){
        SketchGroup* group = new SketchGroup;
        auto line = getExactlyDesiredLine(lines, orientation_tag);
        auto adj_lines = line->getAllAdjLinesByChosenOrient(lines, orientation_tag);
        group->lines.insert(adj_lines.begin(), adj_lines.end());

        /**** orientation tag by if connector components ****/
        if(isStraightLines(adj_lines) && (!isStraightNonOrthorgLine(adj_lines, orientation_tag))){// connector components

            if(adj_lines.size() == 1 && (*adj_lines.begin())->intLength()==2){
                //tiny component
                group->orientation_tag = (*adj_lines.begin())->chosen_orientation;
            }else{
                //connector components
                group->orientation_tag = (*adj_lines.begin())->possi_orientation;
            }

        }else{
            group->orientation_tag = orientation_tag;
        }
        result_group.push_back(group);
        for(auto& l : adj_lines)
            lines.erase(l);
    }

    if(!erase){
        for(auto& group : result_group){
            lines.insert(group->lines.begin(), group->lines.end());
        }
    }
}

SketchLine* SketchGraph::getGeneralizedDesiredLine(const set<SketchLine *>& lines, int line_tag){
    int line_tag1;
    int line_tag2;
    if(line_tag==        Constants::PLANE_XZ){
        line_tag1 =      Constants::PLANE_YZ_OR_XZ;
        line_tag2 =      Constants::PLANE_XY_OR_XZ;
    }else if(line_tag == Constants::PLANE_YZ){
        line_tag1 =      Constants::PLANE_YZ_OR_XZ;
        line_tag2 =      Constants::PLANE_XY_OR_YZ;
    }else if(line_tag == Constants::PLANE_XY){
        line_tag1 =      Constants::PLANE_XY_OR_YZ;
        line_tag2 =      Constants::PLANE_XY_OR_XZ;
    }

    for(auto& l : lines){
        if(l->possi_orientation == line_tag || l->possi_orientation == line_tag1 || l->possi_orientation == line_tag2){
            return l;
        }
    }
    return NULL;
}

SketchLine* SketchGraph::getExactlyDesiredLine(const set<SketchLine*>& lines, int plane_flag){
    for(auto& l : lines){
        if(l->chosen_orientation == plane_flag){
            return l;
        }
    }
    return NULL;
}