//
// Created by 徐豪 on 2017/10/27.
//

#include "util/util.h"
#include "configure/global_variables.h"
#include "BrickFactory.h"
#include "connector/Pin.h"
#include "connector/Connector.h"

using Eigen::Matrix3d;
using Eigen::Vector3d;

BrickFactory::BrickFactory()
{
    initAllBeams();
    initStraightConnectors();
    initNonStraightConnectors();
    assert(!this->beam_list.empty());
    assert(!this->straight_conns.empty());
    assert(!this->nonStraight_conns.empty());
}


const vector<BeamTemplate> &BrickFactory::getAllBeams() const
{
    return beam_list;
}

const vector<Connector> &BrickFactory::getAllStraightConnectors() const {
    assert(!this->straight_conns.empty());
    return this->straight_conns;
}

const vector<Connector> &BrickFactory::getAllNonStraightConnectors() const {
    assert(!this->nonStraight_conns.empty());
    return this->nonStraight_conns;
}

const vector<Connector> BrickFactory::getAllConnectors() const
{
    vector<Connector> conn_list;

    vector<Connector> straight_conn = getAllStraightConnectors();
    conn_list.insert(conn_list.end(), straight_conn.begin(), straight_conn.end());

    vector<Connector> non_straight_conn = getAllNonStraightConnectors();
    conn_list.insert(conn_list.end(), non_straight_conn.begin(), non_straight_conn.end());

    return conn_list;
}

void BrickFactory::initAllBeams() {

    vector<Vector3d> tb1_holes = {Vector3d(0, 0, 0)};
    BeamTemplate TB1("TB1", tb1_holes, "18654.dat", Vector3d(0, 1, 0), true);
    beam_list.push_back(TB1);

    vector<Vector3d> tb2_holes = {Vector3d(0, 0, -0.5), Vector3d(0, 0, 0.5)};
    BeamTemplate TB2("TB2", tb2_holes, "43857.dat", Vector3d(0, 1, 0), true);
    beam_list.push_back(TB2);

    vector<Vector3d> tb3_holes = {Vector3d(0, 0, -1), Vector3d(0, 0, 0), Vector3d(0, 0, 1)};
    BeamTemplate TB3("TB3", tb3_holes, "32523.dat", Vector3d(0, 1, 0), true);
    beam_list.push_back(TB3);

    vector<Vector3d> tb4_holes = {Vector3d(0, 0, -1.5), Vector3d(0, 0, -0.5), Vector3d(0, 0, 0.5), Vector3d(0, 0, 1.5)};
    BeamTemplate TB4("TB4t", tb4_holes, "32449_double.dat", Vector3d(0, 1, 0), true);
    BeamTemplate TB41("TB4_1", tb4_holes, "32449.dat", Vector3d(0, 1, 0), true);
    TB41.translation.y() = 0.25;
    BeamTemplate TB42("TB4_2", tb4_holes, "32449.dat", Vector3d(0, 1, 0), true);
    TB42.translation.y() = -0.25;
    TB4.axle_index = {0, 3};
    TB4.sub_beams = {TB41, TB42};
    beam_list.push_back(TB4);

    vector<Vector3d> tb5_holes = {Vector3d(0, 0, -2), Vector3d(0, 0, -1), Vector3d(0, 0, 0), Vector3d(0, 0, 1),
                                  Vector3d(0, 0, 2)};
    BeamTemplate TB5("TB5", tb5_holes, "32316.dat", Vector3d(0, 1, 0), true);
    beam_list.push_back(TB5);

    vector<Vector3d> tb6t_holes = {Vector3d(0, 0, -2.5), Vector3d(0, 0, -1.5), Vector3d(0, 0, -0.5),
                                   Vector3d(0, 0, 0.5), Vector3d(0, 0, 1.5), Vector3d(0, 0, 2.5)};
    BeamTemplate TB6("TB6", tb6t_holes, "32063_double.dat", Vector3d(0, 1, 0), true);
    BeamTemplate TB61("TB6_1", tb6t_holes, "32063.dat", Vector3d(0, 1, 0), true);
    TB61.translation.y() = 0.25;
    BeamTemplate TB62("TB6_2", tb6t_holes, "32063.dat", Vector3d(0, 1, 0), true);
    TB62.translation.y() = -0.25;
    TB6.sub_beams = {TB61, TB62};
    beam_list.push_back(TB6);

    vector<Vector3d> tb7_holes = {Vector3d(0, 0, -3), Vector3d(0, 0, -2), Vector3d(0, 0, -1), Vector3d(0, 0, 0),
                                  Vector3d(0, 0, 1), Vector3d(0, 0, 2), Vector3d(0, 0, 3)};
    BeamTemplate TB7("TB7", tb7_holes, "32524.dat", Vector3d(0, 1, 0), true);
    beam_list.push_back(TB7);

    vector<Vector3d> tb9_holes = {Vector3d(0, 0, -4), Vector3d(0, 0, -3), Vector3d(0, 0, -2), Vector3d(0, 0, -1),
                                  Vector3d(0, 0, 0), Vector3d(0, 0, 1), Vector3d(0, 0, 2), Vector3d(0, 0, 3),
                                  Vector3d(0, 0, 4)};
    BeamTemplate TB9("TB9", tb9_holes, "40490.dat", Vector3d(0, 1, 0), true);
    beam_list.push_back(TB9);

    vector<Vector3d> tb11_holes = {Vector3d(0, 0, -5), Vector3d(0, 0, -4), Vector3d(0, 0, -3), Vector3d(0, 0, -2),
                                   Vector3d(0, 0, -1), Vector3d(0, 0, 0), Vector3d(0, 0, 1), Vector3d(0, 0, 2),
                                   Vector3d(0, 0, 3), Vector3d(0, 0, 4), Vector3d(0, 0, 5)};
    BeamTemplate TB11("TB11", tb11_holes, "32525.dat", Vector3d(0, 1, 0), true);
    beam_list.push_back(TB11);

    vector<Vector3d> tb13_holes = {Vector3d(0, 0, -6), Vector3d(0, 0, -5), Vector3d(0, 0, -4), Vector3d(0, 0, -3),
                                   Vector3d(0, 0, -2), Vector3d(0, 0, -1), Vector3d(0, 0, 0), Vector3d(0, 0, 1),
                                   Vector3d(0, 0, 2), Vector3d(0, 0, 3), Vector3d(0, 0, 4), Vector3d(0, 0, 5),
                                   Vector3d(0, 0, 6)};
    BeamTemplate TB13("TB13", tb13_holes, "41239.dat", Vector3d(0, 1, 0), true);
    beam_list.push_back(TB13);

    vector<Vector3d> tb15_holes = {Vector3d(0, 0, -7), Vector3d(0, 0, -6), Vector3d(0, 0, -5), Vector3d(0, 0, -4),
                                   Vector3d(0, 0, -3), Vector3d(0, 0, -2), Vector3d(0, 0, -1), Vector3d(0, 0, 0),
                                   Vector3d(0, 0, 1), Vector3d(0, 0, 2), Vector3d(0, 0, 3), Vector3d(0, 0, 4),
                                   Vector3d(0, 0, 5), Vector3d(0, 0, 6), Vector3d(0, 0, 7)};
    BeamTemplate TB15("TB15", tb15_holes, "32278.dat", Vector3d(0, 1, 0), true);
    beam_list.push_back(TB15);

    vector<Vector3d> tb2by4bt90 = {Vector3d(0, 0, 0), Vector3d(0, 0, 1), Vector3d(0, 0, 2), Vector3d(0, 0, 3),
                                   Vector3d(1, 0, 3)};
    BeamTemplate TB2by4("TB2by4Bt90", tb2by4bt90, "32140.dat", Vector3d(0, 1, 0), false);
    TB2by4.axle_index = {0};
    beam_list.push_back(TB2by4);

    vector<Vector3d> tb3by3bt90t_holes = {Vector3d(2, 0, 0), Vector3d(1, 0, 0), Vector3d(0, 0, 0), Vector3d(0, 0, 1),
                                          Vector3d(0, 0, 2)};
    BeamTemplate TB3by3("TB3by3Bt90t", tb3by3bt90t_holes, "32056_double.dat", Vector3d(0, 1, 0), false);
    BeamTemplate TB3by31("TB3by3Bt90t_1", tb3by3bt90t_holes, "32056.dat", Vector3d(0, 1, 0), false);
    TB3by31.translation.y() = 0.25;
    BeamTemplate TB3by32("TB3by3Bt90t_2", tb3by3bt90t_holes, "32056.dat", Vector3d(0, 1, 0), false);
    TB3by32.translation.y() = -0.25;
    TB3by3.axle_index = {0, 2, 4};
    TB3by3.sub_beams = {TB3by31, TB3by32};
    beam_list.push_back(TB3by3);

    vector<Vector3d> tb3by5Bt90_holes = {Vector3d(0, 0, 0), Vector3d(0, 0, 1), Vector3d(0, 0, 2), Vector3d(0, 0, 3),
                                         Vector3d(0, 0, 4), Vector3d(1, 0, 4), Vector3d(2, 0, 4)};
    BeamTemplate TB3by5("TB3by5Bt90", tb3by5Bt90_holes, "32526.dat", Vector3d(0, 1, 0), false);
    beam_list.push_back(TB3by5);

    vector<Vector3d> tb3by7bt53_holes = {Vector3d(0, 0, 0), Vector3d(0, 0, 1), Vector3d(0, 0, 2), Vector3d(0, 0, 3),
                                         Vector3d(0, 0, 4), Vector3d(0, 0, 5), Vector3d(0, 0, 6), Vector3d(0.8, 0, 6.6),
                                         Vector3d(1.6, 0, 7.2)};
    BeamTemplate TB3by7("TB3by7Bt53", tb3by7bt53_holes, "32271.dat", Vector3d(0, 1, 0), false);
    TB3by7.axle_index = {0, (int) tb3by7bt53_holes.size() - 1};
    beam_list.push_back(TB3by7);

    vector<Vector3d> tb4by4bt53_holes = {Vector3d(0, 0, 0), Vector3d(0, 0, 1), Vector3d(0, 0, 2), Vector3d(0, 0, 3),
                                         Vector3d(0.8, 0, 3.6), Vector3d(1.6, 0, 4.2), Vector3d(2.4, 0, 4.8)};
    BeamTemplate TB4bt4("TB4by4Bt53", tb4by4bt53_holes, "32348.dat", Vector3d(0, 1, 0), false);
    TB4bt4.axle_index = {0, (int) tb4by4bt53_holes.size() - 1};
    beam_list.push_back(TB4bt4);

    std::reverse(beam_list.begin(), beam_list.end());
}

void BrickFactory::initStraightConnectors() {

    /****** 2 Pins ******/
    vector<Pin> pin_2780 = {Pin(Vector3d(-0.5, 0, 0), Vector3d(-1, 0, 0), false, true),
                            Pin(Vector3d(0.5, 0, 0), Vector3d(1, 0, 0), false, true)};
    Connector connector_2780("two_pin", pin_2780, "2780.dat");
    connector_2780.isStraight = true;
    straight_conns.push_back(connector_2780);

    /****** 3 Pins ******/
    vector<Pin> pin_6558 = {Pin(Vector3d(-1, 0, 0), Vector3d(1, 0, 0), false, true),
                            Pin(Vector3d(0, 0, 0), Vector3d(1, 0, 0), false, true),
                            Pin(Vector3d(1, 0, 0), Vector3d(1, 0, 0), false, true)};
    Connector connector_6558("three_pin", pin_6558, "6558.dat");
    connector_6558.isStraight = true;
    straight_conns.push_back(connector_6558);

    /****** 4 Pins ******/
    vector<Pin> pin_4p_1o_15462 = {Pin(Vector3d(-2.5, 0, 0), Vector3d(-1, 0, 0), false, false),
                                   Pin(Vector3d(-1.5, 0, 0), Vector3d(-1, 0, 0), true, true),
                                   Pin(Vector3d(-0.5, 0, 0), Vector3d(-1, 0, 0), true, true),
                                   Pin(Vector3d(0.5, 0, 0), Vector3d(1, 0, 0), true, true),
                                   Pin(Vector3d(1.5, 0, 0), Vector3d(1, 0, 0), true, true),
                                   Pin(Vector3d(2.5, 0, 0), Vector3d(-1, 0, 0), false, false)
    };
    Connector connector_pin_4p_1o_15462("4p_1o_15462", pin_4p_1o_15462, "4p_1o_15462.dat");
    connector_pin_4p_1o_15462.isStraight = true;
    straight_conns.push_back(connector_pin_4p_1o_15462);

    /****** 5 Pins ******/
    vector<Pin> pin_5p_1o_3706 = {Pin(Vector3d(-3, 0, 0), Vector3d(-1, 0, 0), false, false),
                                  Pin(Vector3d(-2, 0, 0), Vector3d(-1, 0, 0), true, true),
                                  Pin(Vector3d(-1, 0, 0), Vector3d(-1, 0, 0), true, true),
                                  Pin(Vector3d(-0, 0, 0), Vector3d(-1, 0, 0), true, true),
                                  Pin(Vector3d(1, 0, 0), Vector3d(1, 0, 0), true, true),
                                  Pin(Vector3d(2, 0, 0), Vector3d(1, 0, 0), true, true),
                                  Pin(Vector3d(3, 0, 0), Vector3d(1, 0, 0), false, false)};
    Connector connector_4p_1o_3706("4p_1o_3706", pin_5p_1o_3706, "4p_1o_3706.dat");
    connector_4p_1o_3706.isStraight = true;
    straight_conns.push_back(connector_4p_1o_3706);


    if (glob_vars::current_model.name == "plane") {
        vector<Pin> pin_5p_32073 = {
                Pin(Vector3d(-2, 0, 0), Vector3d(-1, 0, 0), true, true),
                Pin(Vector3d(-1, 0, 0), Vector3d(-1, 0, 0), true, true),
                Pin(Vector3d(-0, 0, 0), Vector3d(-1, 0, 0), true, true),
                Pin(Vector3d(1, 0, 0), Vector3d(1, 0, 0), true, true),
                Pin(Vector3d(2, 0, 0), Vector3d(1, 0, 0), true, true),
        };
        Connector connector_5p_32073("32073", pin_5p_32073, "32073.dat");
        connector_5p_32073.isStraight = true;
        straight_conns.push_back(connector_5p_32073);
    }

    /****** 6 Pins ******/
    vector<Pin> pin_6p_1o_44294 = {Pin(Vector3d(-3, 0, 0), Vector3d(-1, 0, 0), false, false),
                                   Pin(Vector3d(-2.5, 0, 0), Vector3d(-1, 0, 0), true, true),
                                   Pin(Vector3d(-1.5, 0, 0), Vector3d(-1, 0, 0), true, true),
                                   Pin(Vector3d(-0.5, 0, 0), Vector3d(-1, 0, 0), true, true),
                                   Pin(Vector3d(0.5, 0, 0), Vector3d(1, 0, 0), true, true),
                                   Pin(Vector3d(1.5, 0, 0), Vector3d(1, 0, 0), true, true),
                                   Pin(Vector3d(2.5, 0, 0), Vector3d(1, 0, 0), true, true),
                                   Pin(Vector3d(3, 0, 0), Vector3d(1, 0, 0), false, false)};
    Connector connector_6p_1o_44294("6p_1o_44294", pin_6p_1o_44294, "6p_1o_44294.dat");
    connector_6p_1o_44294.isStraight = true;
    straight_conns.push_back(connector_6p_1o_44294);

    /****** 7 Pins ******/
    vector<Pin> pin_7p_1o_3707 = {Pin(Vector3d(-4, 0, 0), Vector3d(-1, 0, 0), false, false),
                                  Pin(Vector3d(-3, 0, 0), Vector3d(-1, 0, 0), true, true),
                                  Pin(Vector3d(-2, 0, 0), Vector3d(-1, 0, 0), true, true),
                                  Pin(Vector3d(-1, 0, 0), Vector3d(-1, 0, 0), true, true),
                                  Pin(Vector3d(-0, 0, 0), Vector3d(-1, 0, 0), true, true),
                                  Pin(Vector3d(1, 0, 0), Vector3d(1, 0, 0), true, true),
                                  Pin(Vector3d(2, 0, 0), Vector3d(1, 0, 0), true, true),
                                  Pin(Vector3d(3, 0, 0), Vector3d(1, 0, 0), true, true),
                                  Pin(Vector3d(4, 0, 0), Vector3d(1, 0, 0), false, false)};
    Connector connector_7p_1o_3707("7p_1o_3707", pin_7p_1o_3707, "7p_1o_3707.dat");
    connector_7p_1o_3707.isStraight = true;
    straight_conns.push_back(connector_7p_1o_3707);

}

void BrickFactory::initNonStraightConnectors() {

    /****** 4 Pins, 1 Orientations(Long axle connector) *******/
    vector<Pin> pin_4p_1o_11214 = {Pin(Vector3d(1, 0, 0), Vector3d(1, 0, 0), false, true),
                                   Pin(Vector3d(0, 0, 0), Vector3d(1, 0, 0), false, true),
                                   Pin(Vector3d(-1, 0, 0), Vector3d(-1, 0, 0), false, false),
                                   Pin(Vector3d(-2, 0, 0), Vector3d(-1, 0, 0), false, true),
                                   Pin(Vector3d(-3, 0, 0), Vector3d(-1, 0, 0), false, true)};
    Connector connector_4p_1o_11214("4p_1o_11214", pin_4p_1o_11214, "4p_1o_11214.dat");
    connector_4p_1o_11214.isStraight = true; // can't cover edges with distance > 1, so set to straight
    nonStraight_conns.push_back(connector_4p_1o_11214);


    /****** 2 Pins, 2 Orientations *******/
    vector<Pin> conn_15100_add1 = {Pin(Vector3d(1, 0, 0), Vector3d(1, 0, 0), false, true),
                                   Pin(Vector3d(0, 0, 0), Vector3d(0, 0, 0), false, false),
                                   Pin(Vector3d(0, 1, 0), Vector3d(0, 1, 0), false, true)};
    Connector connector_15100_add1("15100_add1", conn_15100_add1, "2p_2o_15100.dat");
    nonStraight_conns.push_back(connector_15100_add1);


    vector<Pin> conn_2p_2o_15100_2 = {Pin(Vector3d(1, 0, 0), Vector3d(1, 0, 0), false, true),
                                      Pin(Vector3d(0, 0, 0), Vector3d(1, 0, 0), false, false),
                                      Pin(Vector3d(0, 1, 0), Vector3d(0, 0, 0), false, false),
                                      Pin(Vector3d(1, 1, 0), Vector3d(0, 1, 0), false, false),
                                      Pin(Vector3d(1, 1, -1), Vector3d(0, 0, -1), false, true)};
    Connector connector_2p_2o_15100_2("2p_2o_15100_2", conn_2p_2o_15100_2, "2p_2o_15100_2.dat");
    nonStraight_conns.push_back(connector_2p_2o_15100_2);

    vector<Pin> conn_2p_2o_15100_3 = {Pin(Vector3d(1, 0, 0), Vector3d(1, 0, 0), false, true),
                                      Pin(Vector3d(0, 0, 0), Vector3d(1, 0, 0), false, false),
                                      Pin(Vector3d(0, 1, 0), Vector3d(0, 0, 0), false, false),
                                      Pin(Vector3d(1, 1, 0), Vector3d(0, 1, 0), false, false),
                                      Pin(Vector3d(1, 1, 1), Vector3d(0, 0, 1), false, true)};
    Connector connector_2p_2o_15100_3("2p_2o_15100_3", conn_2p_2o_15100_3, "2p_2o_15100_3.dat");
    nonStraight_conns.push_back(connector_2p_2o_15100_3);

    vector<Pin> conn_6536_add2 = {Pin(Vector3d(0, 1, 1), Vector3d(0, 0, 1), false, true),
                                  Pin(Vector3d(0, 1, 0), Vector3d(0, 0, 0), false, false),
                                  Pin(Vector3d(0, 0, 0), Vector3d(0, 0, 0), false, false),
                                  Pin(Vector3d(-1, 0, 0), Vector3d(-1, 0, 0), false, true)};
    Connector connector_6536_add2("6536_add2", conn_6536_add2, "2p_2o_6536_1.dat");
    nonStraight_conns.push_back(connector_6536_add2);

    vector<Pin> conn_6536_add2_2 = {Pin(Vector3d(0, 1, 1), Vector3d(0, 0, 1), false, true),
                                    Pin(Vector3d(0, 1, 0), Vector3d(0, 0, 0), false, false),
                                    Pin(Vector3d(0, 0, 0), Vector3d(0, 0, 0), false, false),
                                    Pin(Vector3d(1, 0, 0), Vector3d(-1, 0, 0), false, true)};
    Connector connector_6536_add2_2("6536_add2_2", conn_6536_add2_2, "2p_2o_6536_2.dat");
    nonStraight_conns.push_back(connector_6536_add2_2);

    vector<Pin> conn_2p_2o_32523 = {Pin(Vector3d(0, 1, -1), Vector3d(0, 1, 0), false, true),
                                    Pin(Vector3d(0, 0, -1), Vector3d(0, 0, 0), false, false),
                                    Pin(Vector3d(0, 0, 0), Vector3d(0, 0, 0), false, false),
                                    Pin(Vector3d(0, 0, 1), Vector3d(0, 0, 0), false, false),
                                    Pin(Vector3d(0, 1, 1), Vector3d(0, 0, 0), false, false),
                                    Pin(Vector3d(0, 1, 0), Vector3d(0, 0, -1), false, true)};
    Connector connector_2p_2o_32523("2p_2o_32523", conn_2p_2o_32523, "2p_2o_32523.dat");
    nonStraight_conns.push_back(connector_2p_2o_32523);

    vector<Pin> conn_2p_2o_60483 = {Pin(Vector3d(1, -1, 1), Vector3d(0, 0, 1), false, true),
                                    Pin(Vector3d(1, -1, 0), Vector3d(0, 0, 0), false, false),
                                    Pin(Vector3d(0, -1, 0), Vector3d(0, 0, 0), false, false),
                                    Pin(Vector3d(0, 0, 0), Vector3d(0, 0, 0), false, false),
                                    Pin(Vector3d(0, 0, 1), Vector3d(0, 0, 0), false, false),
                                    Pin(Vector3d(0, -1, 1), Vector3d(0, -1, 0), false, true)};
    Connector connector_2p_2o_60483("2p_2o_60483", conn_2p_2o_60483, "2p_2o_60483.dat");
    nonStraight_conns.push_back(connector_2p_2o_60483);

    vector<Pin> pin_2p_2o_15100_4 = {Pin(Vector3d(1, 0, 0), Vector3d(1, 0, 0), false, true),
                                     Pin(Vector3d(0, 0, 0), Vector3d(0, 1, 0), false, false),
                                     Pin(Vector3d(0, 1, 0), Vector3d(0, 1, 0), false, false),
                                     Pin(Vector3d(0, 2, 0), Vector3d(0, 1, 0), false, true)};
    Connector connector_2p_2o_15100_4("2p_2o_15100_4", pin_2p_2o_15100_4, "2p_2o_15100_4.dat");
    nonStraight_conns.push_back(connector_2p_2o_15100_4);


    /****** 3 Pins, 2 Orientations *******/
    vector<Pin> pin_3p_2o_32184 = {Pin(Vector3d(0, 0, 0), Vector3d(0, 0, 1), false, false),
                                   Pin(Vector3d(0, 1, 0), Vector3d(1, 0, 0), false, false),
                                   Pin(Vector3d(0, -1, 0), Vector3d(1, 0, 0), false, false),
                                   Pin(Vector3d(0, 0, 1), Vector3d(0, 0, 1), false, true),
                                   Pin(Vector3d(-1, 1, 0), Vector3d(-1, 0, 0), false, true),
                                   Pin(Vector3d(-1, -1, 0), Vector3d(-1, 0, 0), false, true)};
    Connector connector_3p_2o_32184("3p_2o_32184", pin_3p_2o_32184, "3p_2o_32184.dat");
    vector<std::pair<int, int> > conn_edges_3p_2o_32184 = {{1, 4},
                                                           {0, 1},
                                                           {0, 3},
                                                           {0, 2},
                                                           {2, 5}};
    connector_3p_2o_32184.conn_edges = conn_edges_3p_2o_32184;
    nonStraight_conns.push_back(connector_3p_2o_32184);

    vector<Pin> pin_3p_2o_87082 = {Pin(Vector3d(0, 0, 0), Vector3d(0, 1, 0), false, false),
                                   Pin(Vector3d(1, 0, 0), Vector3d(1, 0, 0), false, true),
                                   Pin(Vector3d(-1, 0, 0), Vector3d(-1, 0, 0), false, true),
                                   Pin(Vector3d(0, 1, 0), Vector3d(0, 1, 0), false, true)};
    Connector connector_3p_2o_87082("3p_2o_87082", pin_3p_2o_87082, "3p_2o_87082.dat");
    vector<std::pair<int, int> > conn_edges_3p_2o_87082 = {{0, 1},
                                                           {0, 2},
                                                           {0, 3}};
    connector_3p_2o_87082.conn_edges = conn_edges_3p_2o_87082;
    nonStraight_conns.push_back(connector_3p_2o_87082);

    vector<Pin> pin_3p_2o_15100 = {Pin(Vector3d(1, 0, 0), Vector3d(1, 0, 0), false, true),
                                   Pin(Vector3d(0, 0, 0), Vector3d(0, 1, 0), false, false),
                                   Pin(Vector3d(0, 1, 0), Vector3d(0, 1, 0), false, true),
                                   Pin(Vector3d(0, 2, 0), Vector3d(0, 1, 0), false, true)};
    Connector connector_3p_2o_15100("3p_2o_15100", pin_3p_2o_15100, "3p_2o_15100.dat");
    nonStraight_conns.push_back(connector_3p_2o_15100);


    vector<Pin> pin_3p_2o_15100_double = {Pin(Vector3d(1, 0, 0), Vector3d(1, 0, 0), false, true),
                                          Pin(Vector3d(0, 0, 0), Vector3d(0, 1, 0), false, false),
                                          Pin(Vector3d(0, 1, 0), Vector3d(0, 1, 0), false, true),
                                          Pin(Vector3d(0, 2, 0), Vector3d(0, 1, 0), false, false),
                                          Pin(Vector3d(1, 2, 0), Vector3d(1, 0, 0), false, true)};
    Connector connector_pin_3p_2o_15100_double("3p_2o_15100_double", pin_3p_2o_15100_double, "3p_2o_15100_double.dat");
    nonStraight_conns.push_back(connector_pin_3p_2o_15100_double);

}

