cmake_minimum_required(VERSION 3.7)
project(lego_technic_system)

INCLUDE_DIRECTORIES(./lib/eigen)
#INCLUDE_DIRECTORIES(../jsoncpp/json)
#link_directories(../jsoncpp/lib_json)


set(SOURCE_FILES core/SketchPoint.h core/SketchLine.h
        bricks/BeamTemplate.cpp bricks/BeamTemplate.h util/util.h core/SketchGraph.cpp
        core/SketchGraph.h core/SketchPoint.cpp core/SketchLine.cpp
        util/util.cpp
        core/LEGOGenerator.cpp core/LEGOGenerator.h core/LayerHole.cpp core/LayerHole.h
        bricks/BrickFactory.cpp bricks/BrickFactory.h objectives/Grader.cpp objectives/Grader.h core/LayerGraph.cpp core/LayerGraph.h core/Model.h configure/global_variables.h hard_code/SymmetryRelation.h configure/global_variables.cpp hard_code/SymmetryRelation.cpp core/SketchGroup.cpp core/SketchGroup.h
        connector/Pin.h connector/Pin.cpp connector/Connector.h connector/Connector.cpp
        bricks/BeamHole.h bricks/BeamHole.cpp
        hard_code/BoundingBox.h hard_code/BoundingBox.cpp post_processing/PostProcessing.cpp core/ConstructionGraph.cpp core/ConstructionGraph.h core/Component.cpp core/Component.h io/DataWriter.cpp io/DataWriter.h ColorTable.cpp ColorTable.h core/UnitEdge.cpp core/UnitEdge.h util/Helper.h core/BeamLayout.cpp core/BeamLayout.h bricks/BeamInstance.cpp bricks/BeamInstance.h bricks/BrickCompare.h
        util/Timer.h util/Timer.cpp objectives/GradeRecorder.h objectives/GradeRecorder.cpp connector/BeamLayoutConnectorGenerator.cpp connector/BeamLayoutConnectorGenerator.h core/UnitEdgeGroup.cpp core/UnitEdgeGroup.h core/GuidingGraph.cpp core/GuidingGraph.h configure/Constants.cpp configure/Constants.h post_processing/PostProcessing.h util/MyAssert.h util/MyAssert.cpp io/DataReader.h io/DataReader.cpp util/Symmetry.h util/Symmetry.cpp)

add_library(lego_technic_system STATIC ${SOURCE_FILES})
#target_link_libraries(lego_technic_system libjsoncpp.a)
