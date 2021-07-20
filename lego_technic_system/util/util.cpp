//
// Created by 徐豪 on 2017/8/8.
//
#include "util/util.h"
#include <iostream>
#include <cmath>
#include <fstream>
#include "bricks/BeamTemplate.h"
#include "core/LayerGraph.h"
#include "configure/global_variables.h"
#include "util/MyAssert.h"
#if defined(_WIN32) || defined(WIN32) || defined(_WIN64) || defined(WIN64)
    #include "bits/stdc++.h"
#endif
#define _USE_MATH_DEFINES

using std::string;


std::string getCurrentTimeAsString()
{
    time_t rawtime;
    struct tm * timeinfo;
    char buffer[80];

    time(&rawtime);
    timeinfo = localtime(&rawtime);

    strftime(buffer,sizeof(buffer),"%m%d-%H%M-%S",timeinfo);
    std::string str(buffer);
    return str;
}

double cRandom(double min, double max)
{
    return min+(max-min)*(rand() / double(RAND_MAX));
}

int intRandom(int min, int max)
{
    return min + (rand() % static_cast<int>(max - min + 1));
}

double length(double p1x, double p1y,double p2x, double p2y){
    return sqrt((p1x-p2x)*(p1x-p2x)+(p1y-p2y)*(p1y-p2y));
}

double length(SketchPoint* p1, SketchPoint* p2){
    return sqrt((p1->_x-p2->_x)*(p1->_x-p2->_x)+(p1->_y-p2->_y)*(p1->_y-p2->_y)+(p1->_z-p2->_z)*(p1->_z-p2->_z));
}

std::string getBeamNo(double length,double precision){
    using std::abs;
    if(abs(length-1.0)<precision){
        return string("43857.dat");
    }else if(abs(length-2.0)<precision){
        return string("32523.dat");
    }else if(abs(length-3.0)<precision){//half beam
        return string("32449.dat");
    }else if(abs(length-4.0)<precision){
        return string("32316.dat");
    }else if(abs(length-5.0)<precision){
        return string("32063.dat");
    }else if(abs(length-6.0)<precision){
        return string("32524.dat");
    }else if(abs(length-8.0)<precision){
        return string("40490.dat");
    }else if(abs(length-10.0)<precision){
        return string("32525.dat");
    }else if(abs(length-12.0)<precision){
        return string("41239.dat");
    }else if(abs(length-14.0)<precision){
        return string("32278.dat");
    }else{
        return string("no matching beam, length="+std::to_string(length));
    }
}

bool beamExists(double length, double precision){
    if(getBeamNo(length,precision)=="no matching beam"){
        return false;
    }else{
        return true;
    }
}

vector<Connector*> intersection(vector<Connector*> v1, vector<Connector*> v2)
{
    vector<Connector*> v3;
    for(int i = 0;i<v1.size() ;i++){
        for(int j = 0;j<v2.size(); j++){
            if(v1.at(i) == v2.at(j)){
                Connector* p = v1.at(i);
                v3.push_back(p);
            }
        }
    }
    return v3;
}


void SplitString(const std::string& s, std::vector<std::string>& v, const std::string& c)
{
    std::string::size_type pos1, pos2;
    pos2 = s.find(c);
    pos1 = 0;
    while(std::string::npos != pos2)
    {
        v.push_back(s.substr(pos1, pos2-pos1));

        pos1 = pos2 + c.size();
        pos2 = s.find(c, pos1);
    }
    if(pos1 != s.length())
        v.push_back(s.substr(pos1));
    //remove empty part
    for(int i = 0 ; i < v.size() ; i++){
        if(v.at(i) == ""){
            v.erase(v.begin()+i);
            i--;
        }
    }
}

SketchGraph* readSketchFromObjFile(string path)
{
    vector<SketchLine*>* lines = new vector<SketchLine*>;
    vector<SketchPoint*>* points = new vector<SketchPoint*>;

    using namespace std;
    vector<string> ss;
    ifstream fin(path);
    printf("%s\n",path.c_str());
    string line;
    bool start_feature = false;
    while (getline(fin,line))
    {
        if (line.compare("") != 0 && line.at(0) == 'v')
        {
            SplitString(line,ss," ");
            points->push_back(new SketchPoint(stod(ss.at(1)),stod(ss.at(2)),stod(ss.at(3))));
            ss.clear();
        }else if(line.compare("") != 0 && line.at(0) == 'l') {
            SplitString(line,ss," ");
            for(int i = 1;i<ss.size()-1;i++)
            {
                lines->push_back(new SketchLine(points->at(stoi(ss.at(i))-1),points->at(stoi(ss.at(i+1))-1),start_feature));
            }
            ss.clear();
        }else if(line.find("#Feature") == 0){
            start_feature = true;
        }
    }
    if(lines->size()==0 || points->size() == 0)
    {
        printf("Reading Problem\n");
        exit(1);
    }
    //Erase vertex floating on the air
    (*points).erase(std::remove_if((*points).begin(),
                                   (*points).end(),
                              [](SketchPoint* p){ return p->lines.empty();}),
                    (*points).end());

    assert(MyAsserts::NoRepeatSketchPoints(*points));
    return new SketchGraph(lines, points);
}

double distance(SketchPoint * p1, SketchPoint *p2)
{
    double x_diff = p1->_x-p2->_x;
    double y_diff = p1->_y-p2->_y;
    double z_diff = p1->_z-p2->_z;
    return sqrt(x_diff*x_diff+y_diff*y_diff+z_diff*z_diff);
}
Vector3d* line_closest(SketchLine *line1, SketchLine *line2)
{
        double EPS = 0.00000001;

        Vector3d delta21;
        delta21.x()=(line1->_p2->_x - line1->_p1->_x);
        delta21.y()=(line1->_p2->_y - line1->_p1->_y);
        delta21.z()=(line1->_p2->_z - line1->_p1->_z);

        Vector3d delta41;
        delta41.x()=(line2->_p2->_x - line2->_p1->_x);
        delta41.y()=(line2->_p2->_y - line2->_p1->_y);
        delta41.z()=(line2->_p2->_z - line2->_p1->_z);

        Vector3d delta13;
        delta13.x()=(line1->_p1->_x-line2->_p1->_x);
        delta13.y()=(line1->_p1->_y-line2->_p1->_y);
        delta13.z()=(line1->_p1->_z-line2->_p1->_z);

        double a = delta21.dot(delta21);
        double b = delta21.dot(delta41);
        double c = delta41.dot(delta41);
        double d = delta21.dot(delta13);
        double e = delta41.dot(delta13);
        double D = a * c - b * b;

        double sc, sN, sD = D;
        double tc, tN, tD = D;

        if (D < EPS)
        {
            sN = 0.0;
            sD = 1.0;
            tN = e;
            tD = c;
        }
        else
        {
            sN = (b * e - c * d);
            tN = (a * e - b * d);
            if (sN < 0.0)
            {
                sN = 0.0;
                tN = e;
                tD = c;
            }
            else if (sN > sD)
            {
                sN = sD;
                tN = e + b;
                tD = c;
            }
        }

        if (tN < 0.0)
        {
            tN = 0.0;

            if (-d < 0.0)
                sN = 0.0;
            else if (-d > a)
                sN = sD;
            else
            {
                sN = -d;
                sD = a;
            }
        }
        else if (tN > tD)
        {
            tN = tD;
            if ((-d + b) < 0.0)
                sN = 0;
            else if ((-d + b) > a)
                sN = sD;
            else
            {
                sN = (-d + b);
                sD = a;
            }
        }

        if (fabs(sN) < EPS) sc = 0.0;
        else sc = sN / sD;
        if (fabs(tN) < EPS) tc = 0.0;
        else tc = tN / tD;

        Vector3d l1_start(line1->_p1->_x,line1->_p1->_y,line1->_p1->_z);
        Vector3d l2_start(line2->_p1->_x,line2->_p1->_y,line2->_p1->_z);

        Vector3d* clos_points = new Vector3d[2];
        clos_points[0] = l1_start+sc*delta21;//closest point on line1
        clos_points[1] = l2_start+tc*delta41;//closest point on line2

        return clos_points;
}


Eigen::MatrixXd getAffine_Mat(vector<SketchLine*> *lines)
{
    Eigen::MatrixXd affin_mat(lines->size(),lines->size());

    for(int i = 0;i<lines->size();i++){
        for(int j = 0;j<lines->size();j++){
            Eigen::Vector3d i_vec = lines->at(i)->getVector();
            Eigen::Vector3d j_vec = lines->at(j)->getVector();
            Vector3d* clos_points = line_closest(lines->at(i), lines->at(j));
            Vector3d v(clos_points[0].x()-clos_points[1].x(),clos_points[0].y()-clos_points[1].y(),clos_points[0].z()-clos_points[1].z());
            double distance = v.norm();
            delete[] clos_points;
            double orien_value = (lines->at(i)->isConnected(lines->at(j))||(distance!=0&&distance<0.8))? (4/M_PI)*fabs(acos(fabs(i_vec.dot(j_vec)/(i_vec.norm()*j_vec.norm())))-M_PI/4):0;
            if(orien_value!=0 && fabs(i_vec.dot(j_vec)/(i_vec.norm()*j_vec.norm()))>1.0)
            {
                orien_value = 1.0;
            }
            affin_mat(i,j) = orien_value;
        }
    }
    return affin_mat;
}

Vector3d getOrienPlaneNormal(SketchGraph *sketchGraph)
{   //TODO: manually result, updata later
    //cube || star
//    return Vector3d(0,0,1);
    //chair
    return Vector3d(0,1,0);
//    arm || bridge || craler crane
//    return Vector3d(1,0,0);

}

//返回与main orientation平面平行的line
vector<vector<SketchLine*>> getMainOrieLines(SketchGraph *sketchGraph, Vector3d normal)
{
    //TODO:#####################manually result############################### update later
    vector<SketchLine*> lines = *sketchGraph->getLines();
    vector<vector<SketchLine*>> planes;
    //cube
//    vector<SketchLine*> plane1;
//    plane1.push_back(lines.at(0));plane1.push_back(lines.at(1));plane1.push_back(lines.at(2));plane1.push_back(lines.at(3));
//    vector<SketchLine*> plane2;
//    plane2.push_back(lines.at(4));plane2.push_back(lines.at(5));plane2.push_back(lines.at(6));plane2.push_back(lines.at(7));
//    vector<vector<SketchLine*>> planes;
//    planes.push_back(plane1);
//    planes.push_back(plane2);
//    return  planes;

    //chair
//    vector<SketchLine*> plane1;
//    plane1.push_back(lines.at(0));plane1.push_back(lines.at(1));plane1.push_back(lines.at(7));plane1.push_back(lines.at(8));
//    vector<SketchLine*> plane2;
//    plane2.push_back(lines.at(3));plane2.push_back(lines.at(4));plane2.push_back(lines.at(6));plane2.push_back(lines.at(9));
//    vector<vector<SketchLine*>> planes;
//    planes.push_back(plane1);
//    planes.push_back(plane2);
//    return planes;

    //crane arm
//    vector<SketchLine*> plane1;
//    vector<SketchLine*> plane2;
//    for(auto line : lines)
//    {
//        if(fabs(line->_p1->_x - 3)<0.3 && fabs(line->_p2->_x - 3)<0.3)
//        {
//            plane1.push_back(line);
//        }
//        if(fabs(line->_p1->_x + 3)<0.01 && fabs(line->_p2->_x + 3)<0.01)
//        {
//            plane2.push_back(line);
//        }
//    }
//    planes.push_back(plane1);
    //bridge shape || star || car
    planes.push_back(lines);

    return planes;
}

std::vector<int> range(int first, int end)
{
    std::vector<int> vec;
    if(end>first)
    {
        for(int i = first; i<=end;i++)
        {
            vec.push_back(i);
        }
    }else if(end<first)
    {
        for(int i = first; i>=end;i--)
        {
            vec.push_back(i);
        }
    }else{
        vec.push_back(first);
    }

    return vec;
}

double distance_point_line(Vector3d point, Vector3d v_start, Vector3d v_end)
{
    Vector3d a_end = v_end-v_start;
    Vector3d a_point = point - v_start;
    if(a_point.dot(a_end)<=0.0)
        return a_point.norm();

    Vector3d a_point_end = point - v_end;
    if(a_point_end.dot(a_end)>=0.0)
        return a_point_end.norm();

    return (a_end.cross(a_point).norm()/a_end.norm());
}

void perm_combination(int index, int set_number, const vector<vector<vector<BeamTemplate *>>> &sets, int *result,
                      vector<vector<BeamTemplate *>> &possible_groups)
{
    if(index == set_number){//产生一个combination
        vector<BeamTemplate *> curr_group;
        for(int i = 0;i<set_number ; i++){
            vector<BeamTemplate *> current_beams = sets.at(i).at(result[i]);
            curr_group.insert(curr_group.end(),current_beams.begin(),current_beams.end());
        }
        possible_groups.push_back(curr_group);
    }else{
        for(int i = 0;i<sets.at(index).size();i++){
            result[index] = i;
            perm_combination(index+1,set_number,sets,result,possible_groups);
        }
    }
}

vector<BeamTemplate *> general_translate(Vector3d trans, vector<BeamTemplate *> &beams) {
    vector<BeamTemplate *> result;
    for (auto &b : beams) {
        BeamTemplate *nb = new BeamTemplate(*b);
        nb->translate(trans);
        result.push_back(nb);
    }
    return result;
}


bool
collide(vector<BeamTemplate *> &group1, vector<BeamTemplate *> &group2)//return if collision exists between two groups
{
    for(auto& b1 : group1){
    auto holes1 = b1->getCurrentComponentHoles();
    for(auto& h1 : holes1){
        for(auto& b2 : group2){
        auto holes2 = b2->getCurrentComponentHoles();
        for(auto& h2 : holes2){
            if((h1-h2).norm()<0.6)
                return true;
        }
        }
    }
    }
    return false;
}

bool collide(BeamTemplate &b, vector<BeamTemplate *> &group2)
{
    auto holes1 = b.getCurrentComponentHoles();
    for(auto& h1 : holes1){
        for(auto& b2 : group2){
            auto holes2 = b2->getCurrentComponentHoles();
            for(auto& h2 : holes2){
                if((h1-h2).norm()<0.8)
                    return true;
            }
        }
    }
    return false;
}

bool isStraightLines(set<SketchLine*> lines){
    Vector3d orient = (*lines.begin())->getVector();
    for(auto& l : lines){
        if(fabs(fabs((orient.dot(l->getVector())/(orient.norm()*l->getVector().norm()))) - 1) > 0.03){
            return false;
        }
    }
    return true;
}

bool isStraightNonOrthorgLine(set<SketchLine*> lines, int plane_tag)//这种情况下说明虽然是straight的但是plane是一定可以确定的
{
    assert(isStraightLines(lines));
    for(auto& l : lines){
        if(l->possi_orientation != plane_tag)
            return false;
    }
    return true;
}

bool isSameSegment(const Vector3d &l1p1, const Vector3d &l1p2, const Vector3d &l2p1, const Vector3d &l2p2){
    if( ( (l1p1-l2p1).norm()<0.03 || (l1p1-l2p2).norm()<0.03) && ((l1p2-l2p1).norm()<0.03 || (l1p2-l2p2).norm()<0.03)){
        return true;
    }
    return false;
}

int min(int a, int b){
    return a<b? a : b;
}

int max(int a, int b){
    return a<b? b : a;
}

Vector3d getNormalFromOrientationTag(int layer_flag){
    switch (layer_flag){
        case Constants::PLANE_XY:{
            return Vector3d(0,0,1);
            break;
        }
        case Constants::PLANE_YZ:{
            return Vector3d(1,0,0);
            break;
        }
        case Constants::PLANE_XZ:{
            return Vector3d(0,1,0);
            break;
        }
        case Constants::PLANE_XY_OR_XZ:{
            return Vector3d(0,1,1);
            break;
        }
        case Constants::PLANE_XY_OR_YZ:{
            return Vector3d(1,0,1);
            break;
        }
        case Constants::PLANE_YZ_OR_XZ:{
            return Vector3d(1,1,0);
            break;
        }
        default:{
            printf("Plane flag error!!!!\n");
            cin.get();
            exit(1);
        }
    }
}

bool layerGeneralizedEqual(int layer, int generalized_layer)
{
    switch (generalized_layer){
        case Constants::PLANE_XY:{
            return layer == generalized_layer;
            break;
        }
        case Constants::PLANE_YZ:{
            return layer == generalized_layer;
            break;
        }
        case Constants::PLANE_XZ:{
            return layer == generalized_layer;
            break;
        }
        case Constants::PLANE_XY_OR_XZ:{
            return (layer == Constants::PLANE_XY || layer == Constants::PLANE_XZ);
            break;
        }
        case Constants::PLANE_XY_OR_YZ:{
            return (layer == Constants::PLANE_XY || layer == Constants::PLANE_YZ);
            break;
        }
        case Constants::PLANE_YZ_OR_XZ:{
            return (layer == Constants::PLANE_YZ || layer == Constants::PLANE_XZ);
            break;
        }
        default:{
            printf("Plane flag error!!!!\n");
            cin.get();
            exit(1);
        }
    }
}

vector<UnitEdge*> getUnitEdgesFromHoles(vector<LayerHole*>& holes)//从layerhole 的集合中得到所有的小edge
{
    set<UnitEdge*, UnitEdge::UnitEdgeIDComp> result;
    for(auto& h : holes){
        result.insert(h->unit_edges.begin(),h->unit_edges.end());
    }
    vector<UnitEdge*> result_vec(result.begin(),result.end());
    result.clear();
    result_vec.erase(
            std::remove_if(
                    result_vec.begin(),
                    result_vec.end(),
                    [&holes](UnitEdge* ue){ return std::find(holes.begin(),holes.end(), ue->hole1) == holes.end()   ||
                                                   std::find(holes.begin(),holes.end(), ue->hole2) == holes.end();
                    }
            ),
            result_vec.end()
    );
    return result_vec;
}

void deleteElements(vector<BeamInstance*>& m_beams)
{
    for(auto& b : m_beams){
        for(auto& bh : b->beamholes){
            delete bh;
        }
        delete(b);
    }
}

int pickWeightedRandElement(vector<int> weight){
    if(weight.size()<=1){
        return 0;
    } else{
        for(int i = 1; i < weight.size(); i++){
            weight[i] = weight[i-1] + weight[i];
        }
    }

    int rand_num = rand() % weight.back();

    auto iter = std::upper_bound(weight.begin(), weight.end(), rand_num);

    return iter-weight.begin();
}

bool contains(const vector<Connector>& conn_vec, const Connector& connector){
    for(const auto & c : conn_vec){
        if(c.equal(connector)){
            return true;
        }
    }
    return false;
}

bool sortLayerByHoleNum(LayerGraph* lg1, LayerGraph* lg2){
    if ( lg1->hole_num() > lg2->hole_num() ){
        return true;
    }else if( lg1->hole_num() < lg2->hole_num() ){
        return false;
    }else{//equal hole number
        return (*lg1->sketchGroup->lines.begin())->_id < (*lg2->sketchGroup->lines.begin())->_id;
    }
}


