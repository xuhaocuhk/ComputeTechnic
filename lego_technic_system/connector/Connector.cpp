//
// Created by 徐豪 on 2017/8/8.
//

#include <fstream>
#include <util/util.h>
#include <configure/global_variables.h>
#include "core/LayerGraph.h"
#include "Connector.h"
#include "bricks/BrickFactory.h"

Connector::Connector(){};

Connector::Connector(string name, vector<Pin> component_pins, string LD_id, int color):
        name(name), pins(component_pins),ldraw_id(LD_id),color(color)
{
    resetTransformation();
}


void Connector::transform(Matrix3d trans)
{
    this->trans_mat = trans * this->trans_mat;
}

void Connector::translate(Vector3d translate)
{
    this->translation = this->translation + translate;
}

void Connector::resetTransformation()
{
    this->trans_mat = Matrix3d::Identity(3,3);
    this->translation = Vector3d(0,0,0);
}

vector<Pin> Connector::getBasicComponentPins() const
{
    return pins;
}

vector<Pin> Connector::getCurrentComponentPins() const
{
    vector<Pin> result;
    for(const auto& p : pins)
    {
        result.push_back(Pin(trans_mat*p.pos + translation , trans_mat*p.orientation , p.isAxle,p.insertable));
    }
    return result;
}

vector<Pin> Connector::getCurrentInsertablePins() const
{
    vector<Pin> result;
    for(auto p : pins)
    {
        if(p.insertable)
        {
            result.push_back(Pin(trans_mat * p.pos + translation, trans_mat*p.orientation, p.isAxle, p.insertable));
        }
    }
    return result;
}

vector<Pin> Connector::getCurrentNonInsertablePins() const
{
    vector<Pin> result;
    for(auto p : pins)
    {
        if(!p.insertable)
        {
            result.push_back(Pin(trans_mat*p.pos + translation, trans_mat*p.orientation, p.isAxle, p.insertable));
        }
    }
    return result;
}

set<BeamTemplate *> Connector::getConnectedBeams() const {
    set<BeamTemplate *> result;
    for(const auto & bh : this->connected_holes){
        result.insert(bh->beam);
    }
    return result;
}

int Connector::pinNum() const
{
    int insertable_num = 0;
    for(const auto & p : pins){
        if(p.insertable)
            insertable_num++;
    }
    return insertable_num;
}

int Connector::non_pinNum() const
{
    int non_insertable_num = 0;
    for(const auto & p : pins){
        if(!p.insertable)
            non_insertable_num++;
    }
    return non_insertable_num;
}

double Connector::pinhead_ratio() const {
    return (double)pinNum() / pins.size();
}

vector< std::pair<int, int> > Connector::getConnectorEdges(){

    vector< std::pair<int, int> > m_conn_edges;

    if(this->conn_edges.empty()){
        for(int i = 0; i<pins.size()-1; i++){

            if( glob_vars::current_model.name == "plane" && this->name == "4p_1o_3706" && (i==0 || i == 5 )  ){
                continue;
            }

            m_conn_edges.push_back(std::make_pair(i, i+1));
        }
    }else{
        return this->conn_edges;
    }
    return m_conn_edges;
}

bool Connector::covers(UnitEdge* unitEdge) //return whether a connectors covers a unit edge
{
    auto curr_pins = this->getCurrentComponentPins();
    auto m_conn_edges = this->getConnectorEdges();

    for(auto& conn_edge : m_conn_edges){

        auto ue_hole1 = unitEdge->hole1->position();
        auto ue_hole2 = unitEdge->hole2->position();

        auto conn_hole1 = (curr_pins[conn_edge.first].pos - ue_hole1).norm() < (curr_pins[conn_edge.second].pos - ue_hole1).norm() ?
                          curr_pins[conn_edge.first].pos : curr_pins[conn_edge.second].pos ;  // connector hole corresponding to ue_hole1

        auto conn_hole2 = (curr_pins[conn_edge.first].pos - ue_hole2).norm() < (curr_pins[conn_edge.second].pos - ue_hole2).norm() ?
                           curr_pins[conn_edge.first].pos : curr_pins[conn_edge.second].pos ;  // connector hole corresponding to ue_hole2

        Vector3d corresp_1 = ue_hole1 - conn_hole1;
        Vector3d corresp_2 = ue_hole2 - conn_hole2;

        if( (corresp_1 - corresp_2).norm() < 5e-2 && (corresp_1.norm() < (isStraight ? 5e-2 : 1.5 + 5e-2)) ){
            return true;
        }
    }

    return false;
}

bool Connector::equal(const Connector& c) const{
    if(this->ldraw_id!=c.ldraw_id)
        return false;

    for(const auto & p1 : getCurrentInsertablePins()){
        bool exist = false;
        for(const auto & p2 : c.getCurrentInsertablePins()){
            if((p1.pos-p2.pos).norm()<0.05 && fabs(fabs(p1.orientation.dot(p2.orientation))-1.0)<0.05){
                exist = true;
                break;
            }
        }
        if(!exist)
            return false;
    }

    for(const auto & p1 : getCurrentNonInsertablePins()){
        bool exist = false;
        for(const auto & p2 : c.getCurrentNonInsertablePins()){
            if((p1.pos-p2.pos).norm()<0.05){
                exist = true;
                break;
            }
        }
        if(!exist)
            return false;
    }

    return true;
}


bool Connector::collision(const Connector& c) const
{
    for(const auto & pos_1 : this->getCurrentComponentPins()){
        for(const auto & pos_2 : c.getCurrentComponentPins()){
            if((pos_1.pos - pos_2.pos).norm()<0.9)
                return true;
        }
    }
    return false;
}

//指长棍类型的Connector
bool Connector::isAxleConnector() const
{
    if(this->ldraw_id == "4p_1o_4519.dat")
        return true;
    return false;
}

bool Connector::isLowerSide(int plane_tag, double plane_value) const//判断此Connector是不是在一个平面的值小的一面
{
    switch(plane_tag){
        case Constants::PLANE_XZ:{
            auto pins = this->getCurrentComponentPins();
            for(auto& pin : pins){
                if(pin.pos.y()>plane_value)
                    return false;
            }
            return true;
            break;
        }
        default:{
            printf("error!!!\n");
            cin.get();
            exit(1);
        }
    }
}

bool Connector::isCrossPlane(int plane_tag, double plane_value) const//判断此Connector是不是跨越了plane
{
    switch(plane_tag){
        case Constants::PLANE_XZ:{
            auto pins = this->getCurrentComponentPins();
            bool low_side = false;
            bool high_side = false;
            for(auto& pin : pins){
                if(pin.pos.y()>plane_value)
                    high_side = true;
                else if(fabs(pin.pos.y()-plane_value)<0.05)
                    return true;
                else{
                    low_side = true;
                }
            }
            if(low_side && high_side){
                return true;
            } else{
                return false;
            }
            break;
        }
        default:{
            printf("error!!!\n");
            cin.get();
            exit(1);
        }
    }
}

void Connector::toLDrawFormat(int m_color) const{
    printf("1 %d %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %s\n",m_color,
           20*translation.x(),20*translation.y(),20*translation.z(),
           trans_mat(0,0),trans_mat(0,1),trans_mat(0,2),
           trans_mat(1,0),trans_mat(1,1),trans_mat(1,2),
           trans_mat(2,0),trans_mat(2,1),trans_mat(2,2),
           ldraw_id.c_str());
}

string Connector::toLDrawFormatAsString() const{
    char buff[200];
    snprintf(buff, sizeof(buff), "1 %d %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %s\n",color,
             20*translation.x(),20*translation.y(),20*translation.z(),
             trans_mat(0,0),trans_mat(0,1),trans_mat(0,2),
             trans_mat(1,0),trans_mat(1,1),trans_mat(1,2),
             trans_mat(2,0),trans_mat(2,1),trans_mat(2,2),
             ldraw_id.c_str());
    std::string buffAsStdStr = buff;
    return buffAsStdStr;
}

void Connector::getSubBricksFromFile(){
    this->sub_bricks.clear();
    using std::ifstream;
    vector<string> ss;
    string path = glob_vars::PRE_FIX + "3_tools/my_ldraw/joints/" + this->ldraw_id;
    ifstream fin(path);
    string line;
    while (getline(fin,line))
    {
        if (line.compare("") != 0 && line.at(0) == '1')
        {
            SplitString(line,ss," ");
            string ld_id = ss.at(14);//The id
            int color = stoi(ss.at(1));
            Vector3d trans_vec = Vector3d(stod(ss.at(2)),stod(ss.at(3)),stod(ss.at(4)))/20;
            Matrix3d trans_mat;
            trans_mat(0,0) = stod(ss.at(5)); trans_mat(0,1) = stod(ss.at(6));trans_mat(0,2) = stod(ss.at(7));
            trans_mat(1,0) = stod(ss.at(8));trans_mat(1,1) = stod(ss.at(9));trans_mat(1,2) = stod(ss.at(10));
            trans_mat(2,0) = stod(ss.at(11));trans_mat(2,1) = stod(ss.at(12));trans_mat(2,2) = stod(ss.at(13));
            IndividualBrick individualBrick{trans_vec, trans_mat, ld_id};
            this->sub_bricks.push_back(individualBrick);
            ss.clear();
        }
    }
}

string Connector::toIndividualLdrawAsString(){
    getSubBricksFromFile();
    if(this->sub_bricks.empty()){
//        printf("0 connector without individual bricks! ldrawID:%s \n", this->ldraw_id.c_str());
        return this->toLDrawFormatAsString();
    }else{
        string result;
        for(auto b : sub_bricks){
            b.translation = this->translation + this->trans_mat * b.translation;
            b.trans_mat = this->trans_mat * b.trans_mat;
            result += b.toLDrawFormatAsString(color);
        }
        return result;
    }
}