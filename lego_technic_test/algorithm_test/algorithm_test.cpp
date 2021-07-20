////
//// Created by 徐豪 on 2018/6/12.
////
//
//#include <core/DataWriter.h>
//#include <configure/global_variables.h>
//#include <core/LEGOGenerator.h>
//#include <bricks/BrickFactory.h>
//#include <connector_assign/ConnectorGenerator.h>
////#include <connector_assign/ConnectorGenerator.h>
//#include <core/Grader.h>
//#include "gtest/gtest.h"
//
//
//class AlgorithmTest : public ::testing::Test {
//protected:
//    virtual void SetUp() {
//        glob_vars::current_input = 1;
//
//    }
//
//    virtual void TearDown() {
//
//    }
//};
//
//
//TEST_F(AlgorithmTest, test_algorithm) {
//    LEGOGenerator generator;
//    DataWriter::initDebugDirectory();
//}
//
//TEST_F(AlgorithmTest, test_write_beams) {//测试输出individual brick
//    printf("sss\n");
//    BrickFactory brickFactory;
//    auto beams = brickFactory.readBeamsFromFile("/Users/xuhao/Dropbox/LEGO_Technic_data/2_test/test_out_Indiv_bricks.ldr");
//    for(auto& b : beams){
//        std::cout<<b->toIndividualLDrawAsString(4);
//    }
//}
//
//
//TEST_F(AlgorithmTest, test_write_indiv_connectors) {//测试输出individual brick
//    printf("sss\n");
//    BrickFactory brickFactory;
//    auto connectors = brickFactory.readConnectorsFromFile("/Users/xuhao/Dropbox/LEGO_Technic_data/2_test/test_out_Indiv_connectors.ldr");
//    for(auto& c : connectors){
//        std::cout<<c->toIndividualLdrawAsString();
//    }
//}
//
//
//TEST_F(AlgorithmTest, test_connector_assign) {//测试assign connector的算法
//    LEGOGenerator generator;
//    DataWriter::initDebugDirectory();
//
//    BrickFactory brickFactory;
//    auto m_beams = brickFactory.readBeamsFromFile("/Users/xuhao/Dropbox/LEGO_Technic_data/1_debug/plane_1008-2129-08/plane_beams_3DBeamsFinal_26.365357.ldr");
//    ConnectorGenerator connectorGenerator(m_beams);
//    vector<Connector*> connectors = connectorGenerator.genConnectors();
//
//    DataWriter::writeConstructionToFile(m_beams, connectors, 0.0, "testing");
//
//    for(auto& c : connectors){
//        std::cout<<c->toIndividualLdrawAsString();
//    }
//}
//
//
//TEST_F(AlgorithmTest, test_3D_grader) {//测试3D objective function的计算是否正确
//    LEGOGenerator generator;
//    BrickFactory brickFactory;
//    glob_vars::sketchGraph->genUnitEdges();
//    auto m_beams = brickFactory.readBeamsFromFile(glob_vars::PRE_FIX + "2_test/test_3d_objfunc.ldr");
//    auto m_connectors = brickFactory.readConnectorsFromFile(glob_vars::PRE_FIX + "2_test/test_3d_objfunc.ldr");
//    Grader::overall_score(m_beams, m_connectors);
//
//}
//
