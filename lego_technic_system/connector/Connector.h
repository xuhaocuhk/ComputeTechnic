//
// Created by 徐豪 on 2017/8/8.
//

#ifndef LEGOGENERATIONFROMSKETCH_ONEPLANE_PIN_H
#define LEGOGENERATIONFROMSKETCH_ONEPLANE_PIN_H

#include <vector>
#include <set>
#include "bricks/BeamTemplate.h"
#include "Pin.h"
#include "bricks/BeamTemplate.h"
#include "bricks/BeamHole.h"

using std::vector;
using std::string;
using std::endl;
using std::cin;
using std::set;

class BeamTemplate;
class BeamHole;
class UnitEdge;

struct IndividualBrick{
    Vector3d translation;
    Matrix3d trans_mat;
    string ldraw_id;

    string toLDrawFormatAsString(int color){
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
};

class Connector {

public:
    Connector();
    Connector(string name,vector<Pin> component_pins,string LD_id,int color = 14);
    /***** Transformation *****/
    void transform(Matrix3d trans);
    void translate(Vector3d translate);
    void resetTransformation();

    /***** Basic Operation *****/
    vector<Pin> getBasicComponentPins() const;
    vector<Pin> getCurrentComponentPins() const;
    vector<Pin> getCurrentInsertablePins() const;
    vector<Pin> getCurrentNonInsertablePins() const;

    set<BeamTemplate *> getConnectedBeams() const;
    int pinNum() const;
    int non_pinNum() const;
    vector< std::pair<int, int> > getConnectorEdges();
    bool covers(UnitEdge*); //return whether a connectors covers a unit edge

    double pinhead_ratio() const;//有效的insertable的pin占总pin 的比例

    /***** Comparation ******/
    bool equal(const Connector&) const;
    bool collision(const Connector&) const;
    bool isAxleConnector() const;
    bool isLowerSide(int plane_tag, double plane_value) const;//判断此Connector是不是在一个平面的值小的一面
    bool isCrossPlane(int plane_tag, double plane_value) const;//判断此Connector是不是跨越了plane

    /***** Deubg information *****/
    void toLDrawFormat(int color = 14) const;
    string toLDrawFormatAsString() const;
    void getSubBricksFromFile();//从文件中得到所有的subbricks
    string toIndividualLdrawAsString();

public:

    int _id;
    static int max_id;
    int color;
    bool isStraight = false; // is a connector that straightly passing through

    string name;  //Beam的分类名称
    string ldraw_id;   //在ldraw的.dat的名称


    Vector3d translation;
    Matrix3d trans_mat;

    vector<BeamHole*> connected_holes;

    vector<IndividualBrick> sub_bricks;

    vector<Pin> pins;

    vector<UnitEdge*> covered_edges; // unit edges this connector covers

    vector< std::pair<int,int> > conn_edges; //underlying edges of this connector
};


#endif //LEGOGENERATIONFROMSKETCH_ONEPLANE_PIN_H
