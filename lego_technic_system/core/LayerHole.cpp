//
// Created by 徐豪 on 2017/10/14.
//
#define _USE_MATH_DEFINES

#include <util/util.h>
#include <util/Symmetry.h>
#include "LayerGraph.h"
#include "LayerHole.h"


int LayerHole::max_id = 0;

LayerHole::LayerHole(double x, double y, double z, int orient)
{
    this->_id = max_id++;
    _x = x; _y = y; _z= z;
    this->orientation = orient;
    this->symmtry_hole = NULL;
}

void LayerHole::add_adj_hole(LayerHole* h)
{
    adj_holes.insert(h);
}

void LayerHole::addBeam(BeamTemplate *beam)
{
    added_beams.insert(beam);
}

bool LayerHole::fully_covered() const
{
    if(adj_holes.empty()){
        if(this->added_beams.empty())
            return false;
        else
            return true;
    }else{
        for(auto adj_hole: adj_holes)
        {
            bool exist = false;
            for(auto adj_beam : adj_hole->added_beams)
            {
                for(auto beam:added_beams)
                {
                    if(adj_beam == beam){
                        exist = true;
                    }
                }
            }
            if(!exist){
                return false;
            }
        }
        return true;
    }
}

int LayerHole::cover_status()
{
    int cover_count = 0;
    for(auto adj_hole: adj_holes)
    {
        for(auto adj_beam : adj_hole->added_beams)
        {
            for(auto beam:added_beams)
            {
                if(adj_beam == beam){
                    cover_count++;
                }
            }
        }
    }
    return cover_count;
}

bool LayerHole::is_boundary_corner()
{
    if(adj_holes.size() == 1 || adj_holes.size() == 0)
        return true;
    if(adj_holes.size() == 2)
    {
        auto iter = adj_holes.begin();
        Vector3d pos_1 = (*iter++)->position();
        Vector3d pos_2 = (*iter)->position();
        using std::cout;
//        cout<<pos_1<<endl;
//        cout<<pos_2<<endl;
//        cout<<()<<endl;
//        cout<<abs((pos_1 - this->positon()).dot(pos_2 - this->positon()))<<endl;
        if(fabs((pos_1 - this->position()).dot(pos_2 - this->position()))<0.05)
        {
            return true;
        }
    }
//    if(adj_holes.size() == 3)
//    {
//        auto iter = adj_holes.begin();
//        Vector3d pos_1 = (*iter++)->position();
//        Vector3d pos_2 = (*iter++)->position();
//        Vector3d pos_3 = (*iter)->position();
//        if(fabs(fabs((pos_1 - this->position()).dot(pos_2 - this->position()))-1)<0.05 ||
//           fabs(fabs((pos_1 - this->position()).dot(pos_3 - this->position()))-1)<0.05 ||
//           fabs(fabs((pos_2 - this->position()).dot(pos_3 - this->position()))-1)<0.05)//共线
//        {
//            return false;
//        }
//        return true;
//    }
    return false;
}

bool LayerHole::is_T_joint()
{
    if(adj_holes.size() >= 3)
    {
        auto first = adj_holes.begin();
        for(auto iter = adj_holes.begin(); iter != adj_holes.end(); iter++)
        {
            if(fabs(fabs(((*first)->position() - this->position()).dot((*iter)->position() - this->position()))-1)>0.1)
            {
                return true;
            }
        }
    }
    return false;
}

void LayerHole::print()
{
    printf("0 Hole:(%lf,%lf,%lf),Beam Number:%d,Covered:%d, isCorner:%d, isTJoint%d\n",round(_x),round(_y),round(_z),(int)added_beams.size(),fully_covered(),is_boundary_corner(),is_T_joint());
}

void LayerHole::toLdrawFormat(int color)
{
    //根据normal计算rotation矩阵
    Vector3d v1 = Vector3d(0,1,0);
    Vector3d v2 = getNormalFromOrientationTag(this->orientation);
    Vector3d cross_v1_v2 = v1.cross(v2);
    Matrix3d rot_mat;
    if(cross_v1_v2.norm() == 0)
        rot_mat = Matrix3d::Identity(3,3);
    else{
        rot_mat = Eigen::AngleAxisd(acos(v1.dot(v2)/(v1.norm()*v2.norm())), cross_v1_v2/cross_v1_v2.norm()).matrix();
    }
    printf("1 %d %lf %lf %lf %lf  %lf %lf %lf %lf %lf %lf %lf %lf 18654.dat\n",(this->featured && color==11)? 4 : color, this->_x*20, this->_y*20, this->_z * 20,
            rot_mat(0,0),rot_mat(0,1),rot_mat(0,2),
            rot_mat(1,0),rot_mat(1,1),rot_mat(1,2),
            rot_mat(2,0),rot_mat(2,1),rot_mat(2,2));
}

BeamTemplate LayerHole::getUnitHole(Vector3d pos, int orien_tag)//得到在这个位置的18654的hole
{
    assert(orien_tag == Constants::PLANE_XY || orien_tag == Constants::PLANE_XZ || orien_tag == Constants::PLANE_YZ);
    vector<Vector3d> tb1_holes = { Vector3d(0,0,0) };
    BeamTemplate TB1("TB1", tb1_holes, "18654.dat", Vector3d(0, 1, 0), true);
    Matrix3d base_trans;
    switch(orien_tag){
        case Constants::PLANE_XY :
            base_trans = Eigen::AngleAxisd(M_PI/2, Vector3d(1, 0, 0)).matrix();
            break;
        case Constants::PLANE_XZ :
            base_trans = Matrix3d::Identity(3,3);
            break;
        case Constants::PLANE_YZ :
            base_trans = Eigen::AngleAxisd(M_PI/2, Vector3d(0, 0, 1)).matrix();
            break;
        default:
            printf("Layer flag error!!!!!\n");
            exit(1);
    }

    TB1.transform(base_trans);
    TB1.translation = pos;
    TB1.layer = 0;
    TB1.color = 6;
    return TB1;
}

std::string LayerHole::toLdrawFormatAsString(int color){

    //根据normal计算rotation矩阵
    Vector3d v1 = Vector3d(0,1,0);
    Vector3d v2 = getNormalFromOrientationTag(this->orientation);
    Vector3d cross_v1_v2 = v1.cross(v2);
    Matrix3d rot_mat;
    if(cross_v1_v2.norm() == 0)
        rot_mat = Matrix3d::Identity(3,3);
    else{
        rot_mat = Eigen::AngleAxisd(acos(v1.dot(v2)/(v1.norm()*v2.norm())), cross_v1_v2/cross_v1_v2.norm()).matrix();
    }
    char buff[200];
    snprintf(buff, sizeof(buff), "1 %d %lf %lf %lf %lf  %lf %lf %lf %lf %lf %lf %lf %lf 18654.dat\n",(this->featured && color==11)? 4 : color, this->_x*20, this->_y*20, this->_z * 20,
           rot_mat(0,0),rot_mat(0,1),rot_mat(0,2),
           rot_mat(1,0),rot_mat(1,1),rot_mat(1,2),
           rot_mat(2,0),rot_mat(2,1),rot_mat(2,2));
    std::string buffAsStdStr = buff;
    return buffAsStdStr;
}

void DFS_symmetry_holes(LayerHole* layerHole, set<LayerHole*>& result ){
    result.insert(layerHole);
    set<LayerHole*> adj_groups;

    if( layerHole->symmetry_hole_x != NULL ) adj_groups.insert(layerHole->symmetry_hole_x);
    if( layerHole->symmetry_hole_y != NULL ) adj_groups.insert(layerHole->symmetry_hole_y);
    if( layerHole->symmetry_hole_z != NULL ) adj_groups.insert(layerHole->symmetry_hole_z);

    for(auto& h : adj_groups){
        if(result.find(h) == result.end())
            DFS_symmetry_holes(h, result);
    }
}

set<LayerHole*> LayerHole::getAllSymmetryHoles(){

    set<LayerHole*> result;
    DFS_symmetry_holes(this, result);
    return result;
}

void DFS_symmetry_groups_with_dirc(tuple<LayerHole*, vector<int>> & g, set<tuple<LayerHole*,
                                vector<int>>>& result, LayerHole* current_group){
    if(std::get<0>(g) != current_group){
        result.insert(g);
    }
    vector<int> last_direction = std::get<1>(g);
    set<tuple<LayerHole*,vector<int>>> adj_groups;

    if(std::get<0>(g)->symmetry_hole_x != NULL){
        vector<int> current_direc = last_direction;
        current_direc.push_back(Constants::X_AXIS);
        adj_groups.insert(std::make_tuple(std::get<0>(g)->symmetry_hole_x, current_direc));
    }
    if(std::get<0>(g)->symmetry_hole_y != NULL){
        vector<int> current_direc = last_direction;
        current_direc.push_back(Constants::Y_AXIS);
        adj_groups.insert(std::make_tuple(std::get<0>(g)->symmetry_hole_y, current_direc));
    }
    if(std::get<0>(g)->symmetry_hole_z != NULL){
        vector<int> current_direc = last_direction;
        current_direc.push_back(Constants::Z_AXIS);
        adj_groups.insert(std::make_tuple(std::get<0>(g)->symmetry_hole_z, current_direc));
    }
    for(auto e : adj_groups){
        if(std::find_if(result.begin(),result.end(), [&e](const tuple<LayerHole*, vector<int>>& tuple1)
        { return std::get<0>(tuple1) == std::get<0>(e); })
           == result.end() && std::get<0>(e) != current_group)
            DFS_symmetry_groups_with_dirc(e,result,current_group);
    }

}

set< tuple<LayerHole*, vector<int>>> LayerHole::getAllSymmetryLayerHolesWithDirecTag(){
    set< tuple<LayerHole*, vector<int> >> result;
    tuple<LayerHole*, vector<int> > this_tuple = std::make_tuple(this,vector<int>());

    DFS_symmetry_groups_with_dirc(this_tuple, result, this);

    return result;
}

vector<BeamHole*> LayerHole::getBeamHolesOnThisLayerHole(){

    vector<BeamHole*> result_beamholes;
    if(this->belonged_beamLayout != NULL){
        Vector3d plane_normal = this->belonged_beamLayout->belonged_component->layerGraph->getCurrentPlaneNormal();

        for(auto& bi : this->belonged_beamLayout->beamsOnHole[this]) {

            BeamHole *beamHole = new BeamHole( this->position() + bi->layer_number * plane_normal,
                                               false, //TODO: implement a detailed version when we have time
                                               plane_normal, bi->layer_number);
            beamHole->beamInstance = bi;
            bi->beamholes.push_back(beamHole);

            result_beamholes.push_back(beamHole);

            if( this->belonged_beamLayout->belonged_component->layerGraph->sketchGroup->isSelfSymmetry() ){
                if (bi->layer_number != 0) {
                    bool collision = false;

                    for (auto &h : bi->template_beam->covered_holes) {
                        if (std::find_if(this->belonged_beamLayout->beamsOnHole[h].begin(),
                                         this->belonged_beamLayout->beamsOnHole[h].end(),
                                         [bi](BeamInstance *beamInstance) {
                                             return beamInstance->layer_number == -bi->layer_number;
                                         })
                            != this->belonged_beamLayout->beamsOnHole[h].end()) {
                            collision = true;
                            break;
                        }
                    }

                    if (!collision) {
                        BeamHole *beamHole = new BeamHole(this->position() - bi->layer_number * plane_normal,
                                                          false, //TODO: implement a detailed version when we have time
                                                          plane_normal, -bi->layer_number);

                        beamHole->beamInstance = bi;
                        bi->beamholes.push_back(beamHole);

                        result_beamholes.push_back(beamHole);

                    }
                }
            }

        }
        return result_beamholes;

    }else{

        auto symm_holes = this->getAllSymmetryLayerHolesWithDirecTag();

        ////debugging....
//        this->toLdrawFormat(4);
//        for(auto& symm_h : symm_holes){
//            std::get<0>(symm_h)->toLdrawFormat(5);
//        }

//        assert(!symm_holes.empty());
        for(auto& h : symm_holes){
            if( std::get<0>(h)->belonged_beamLayout != NULL ){

                result_beamholes = std::get<0>(h)->getBeamHolesOnThisLayerHole();
                /////debugging
//                for(auto& bhh : result_beamholes){
//                    printf("(%lf %lf %lf) \n", bhh->pos.x(), bhh->pos.y(), bhh->pos.z());
//                }
//                printf("----------\n");

                vector<int> symm_directions = std::get<1>(h);

                for(int direction : symm_directions){
                    result_beamholes = Symmetry::get_all_reflect_beamholes_general(result_beamholes, direction,
                                                                                   std::get<0>(
                                                                                           h)->belonged_beamLayout->belonged_component->layerGraph->getCurrentPlaneFlag());

                    //// debugging
//                    for(auto& bhh : result_beamholes){
//                        printf("(%lf %lf %lf) \n", bhh->pos.x(), bhh->pos.y(), bhh->pos.z());
//                    }
//                    printf("-----\n");
                }

//                for(auto& bhh : result_beamholes){
//                    printf("(%lf %lf %lf) \n", bhh->pos.x(), bhh->pos.y(), bhh->pos.z());
//                }
//                printf("---- end ----\n");
                return result_beamholes;
            }
        }

        return result_beamholes;
    }

}

vector<BeamHole*> LayerHole::getBeamHolesOnthisLayerHoleGivenBeam(BeamInstance* bi){

    vector<BeamHole*> result_beamholes;

    if(this->belonged_beamLayout != NULL){
        Vector3d plane_normal = this->belonged_beamLayout->belonged_component->layerGraph->getCurrentPlaneNormal();

        if( this->belonged_beamLayout->beamsOnHole[this].find(bi) == this->belonged_beamLayout->beamsOnHole[this].end() ){
            printf("---problem here !!!\n");
            std::cout << bi->toLdrawFormatAsString();
            this->toLdrawFormat();
        }

        BeamHole *beamHole = new BeamHole( this->position() + bi->layer_number * plane_normal,
                                           false, //TODO: implement a detailed version when we have time
                                           plane_normal, bi->layer_number);
        beamHole->beamInstance = bi;
        bi->beamholes.push_back(beamHole);

        result_beamholes.push_back(beamHole);

        if( this->belonged_beamLayout->belonged_component->layerGraph->sketchGroup->isSelfSymmetry() ){
            if (bi->layer_number != 0) {
                bool collision = false;

                for (auto &h : bi->template_beam->covered_holes) {
                    if (std::find_if(this->belonged_beamLayout->beamsOnHole[h].begin(),
                                     this->belonged_beamLayout->beamsOnHole[h].end(),
                                     [bi](BeamInstance *beamInstance) {
                                         return beamInstance->layer_number == -bi->layer_number;
                                     })
                        != this->belonged_beamLayout->beamsOnHole[h].end()) {
                        collision = true;
                        break;
                    }
                }

                if (!collision) {
                    BeamHole *beamHole = new BeamHole(this->position() - bi->layer_number * plane_normal,
                                                      false, //TODO: implement a detailed version when we have time
                                                      plane_normal, -bi->layer_number);

                    beamHole->beamInstance = bi;
                    bi->beamholes.push_back(beamHole);

                    result_beamholes.push_back(beamHole);

                }
            }
        }

        return result_beamholes;

    }else{

        auto symm_holes = this->getAllSymmetryLayerHolesWithDirecTag();

        ////debugging....
//        this->toLdrawFormat(4);
//        for(auto& symm_h : symm_holes){
//            std::get<0>(symm_h)->toLdrawFormat(5);
//        }

//        assert(!symm_holes.empty());
        for(auto& h : symm_holes){
            if( std::get<0>(h)->belonged_beamLayout != NULL ){

                result_beamholes = std::get<0>(h)->getBeamHolesOnthisLayerHoleGivenBeam( bi );
                /////debugging
//                for(auto& bhh : result_beamholes){
//                    printf("(%lf %lf %lf) \n", bhh->pos.x(), bhh->pos.y(), bhh->pos.z());
//                }
//                printf("----------\n");

                vector<int> symm_directions = std::get<1>(h);

                for(int direction : symm_directions){
                    result_beamholes = Symmetry::get_all_reflect_beamholes_general(result_beamholes, direction,
                                                                                   std::get<0>(
                                                                                           h)->belonged_beamLayout->belonged_component->layerGraph->getCurrentPlaneFlag());

                    //// debugging
//                    for(auto& bhh : result_beamholes){
//                        printf("(%lf %lf %lf) \n", bhh->pos.x(), bhh->pos.y(), bhh->pos.z());
//                    }
//                    printf("-----\n");
                }

//                for(auto& bhh : result_beamholes){
//                    printf("(%lf %lf %lf) \n", bhh->pos.x(), bhh->pos.y(), bhh->pos.z());
//                }
//                printf("---- end ----\n");
                return result_beamholes;
            }
        }

        return result_beamholes;
    }

}









