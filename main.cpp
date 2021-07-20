#include <iostream>
#include <bricks/BrickFactory.h>
#include <configure/global_variables.h>
#include <objectives/Grader.h>
#include <fstream>
#include <connector/BeamLayoutConnectorGenerator.h>
#include <util/Timer.h>
#include <IO/DataWriter.h>
#include "core/LEGOGenerator.h"

using std::vector;
using std::set;
using std::cout;
using std::endl;
using std::cin;


int main(int argc, char *argv[]) {

    if(argc == 1){
        printf("Please provide sketch path mode!\n");
        printf("-path_select or -path_provide\n");
    }else{
        if( strcmp(argv[1], "-path_select") ==0 ){
            glob_vars::current_input = std::stoi(argv[2]);
        }else if( strcmp(argv[1], "-path_provide") == 0){
            printf("please provide sketth path:\n");
            glob_vars::sketch_path = argv[2];
        }else{
            printf("argument error!\n");
            exit(1);
        }
    }

    LEGOGenerator generator;
    DataWriter::initDebugDirectory();
    DataWriter::writeProgramStatistics(glob_vars::config_file);

    ////establish symmetry on sketchline level
    glob_vars::sketchGraph->estabilshSketchSymmetry();

    ////initialize guiding graph
    glob_vars::guidingGraph = new GuidingGraph(glob_vars::sketchGraph);

    ////statistics of input sketch
    DataWriter::writeInputStatistics(glob_vars::config_file);

    ////stage one:estimate local beam orientation
    glob_vars::sketchGraph->estimateLocalBeamOrientation();

    ////extract components and compute all possible beam placements
    generator.extractComponents();

    generator.computeBeamPlacements();

    ////stage two: main generation
    generator.stage_two_generation();

    /******* Debugging... Show all uncovered edges *******/
    DataWriter::writeUnCoveredUnitEdge( "uncovered_edges" );

    /***** Testing Connector Stuff *****/
    BeamLayoutConnectorGenerator connectorGenerator(generator.major_compont_graph);
    connectorGenerator.genConnectorsCoverUnCoveredEdges();
    connectorGenerator.genStraightConnectors();


    DataWriter::writeBeamHoles(connectorGenerator, "");
    DataWriter::writeBeamInstances(*generator.major_compont_graph, connectorGenerator, "_3DBeamsFinalConnectors_indv_nsymm", true, false);
    DataWriter::writeFinalResultStatistics(*generator.major_compont_graph, connectorGenerator);
    DataWriter::writeFinalResultComplexity(*generator.major_compont_graph, connectorGenerator);

    glob_vars::config_file.close();

    return 0;
}

