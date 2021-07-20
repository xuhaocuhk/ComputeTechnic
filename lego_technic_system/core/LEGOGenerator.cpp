//
// Created by 徐豪 on 2017/10/13.
//

#include "SketchGraph.h"
#include "LEGOGenerator.h"
#include <queue>
#include <stdio.h>
#include <math.h>
#include <map>
#include <list>
#include <algorithm>
#include "bricks/BrickFactory.h"
#include "objectives/Grader.h"
#include <fstream>
#include "configure/global_variables.h"
#include "hard_code/SymmetryRelation.h"
#include "Model.h"
#include <dirent.h>
#include <core/UnitEdge.h>
#include <ctime>
#include "IO/DataWriter.h"
#include "hard_code/BoundingBox.h"

using std::cout;
using std::cin;
using std::endl;
using std::list;
using std::ifstream;


/*
 * ########################### Implementation for class GridSystem #####################################
 */


LEGOGenerator::LEGOGenerator()
{
    setbuf(stdout, NULL);//to print instantly
    if(glob_vars::current_input != -1){
        this->chooseModel();
        glob_vars::sketch_path = glob_vars::PRE_FIX + glob_vars::current_model.sketch_path;
    } else{
        glob_vars::current_model = config::model_names.at(0);
    }
    assert( !glob_vars::sketch_path.empty() );
    printf("Reading sketch %s. \n", glob_vars::sketch_path.c_str());
    glob_vars::sketchGraph = readSketchFromObjFile(glob_vars::sketch_path);

    srand(glob_vars::current_model.seed_number);

    glob_vars::brickFactory = new BrickFactory();
}

void LEGOGenerator::extractComponents(){
    /***** Start grouping ******/
    set<SketchLine*> lines((*(glob_vars::sketchGraph->m_lines)).begin(),(*glob_vars::sketchGraph->m_lines).end());

    ///////////////////////////////////// -> checked here

    glob_vars::sketchGraph->groupLinesByChosenDirection(lines, glob_vars::sketchGraph->all_groups);

    glob_vars::sketchGraph->establishSymmetryGroupRelation( glob_vars::sketchGraph->all_groups);//建立单边的symmetry

    glob_vars::sketchGraph->eraseSymmetryGroups( glob_vars::sketchGraph->all_groups);

    ////holization
    vector<LayerGraph*> all_components;
    for(SketchGroup* group : glob_vars::sketchGraph->all_groups){
        LayerGraph* layer = new LayerGraph(group);
        if(!layer->holes.empty())
            all_components.push_back(layer);
    }

    //Sorting to make the order be unique
    std::sort(all_components.begin(), all_components.end(), sortLayerByHoleNum);

    DataWriter::writeGroupingResult_debug(all_components, "");

    glob_vars::config_file << "Number of groups after symmetry: \t" << all_components.size() << endl;

    int real_group_size = 0;
    for(auto& m : all_components)
        real_group_size += m->sketchGroup->getAllSymmetryGroups().size();

    glob_vars::config_file << "Number of original groups \t" << real_group_size << endl;

    /***** Establish graph relations among all components *****/
    this->major_compont_graph = new ConstructionGraph(all_components);
}

void LEGOGenerator::computeBeamPlacements(){
    /***** Initalize all components *****/
    clock_t start = clock();

    for(auto& component : this->major_compont_graph->components){
        component->SAInit();

        set<UnitEdge*, UnitEdge::UnitEdgeIDComp> u_edges(component->layerGraph->unit_edges.begin(),
                                                         component->layerGraph->unit_edges.end());

        DataWriter::writeUnitEdges( u_edges,
                                    glob_vars::guidingGraph->m_holes,
                                    std::to_string(component->_id) , true);
    }

    //////Update BeamHole Information
    this->major_compont_graph->beamLayoutConnectorGenerator = new BeamLayoutConnectorGenerator(this->major_compont_graph);
    BeamLayoutConnectorGenerator temp_generator( this->major_compont_graph );
    printf("curr_beamholes_size:%d \t real_beamholes_size:%d \n", temp_generator.beamholes.size(), this->major_compont_graph->beamLayoutConnectorGenerator->beamholes.size());
    printf("curr_collision_size:%d \t real_collision_size:%d \n", temp_generator.getCollisionNum(), this->major_compont_graph->beamLayoutConnectorGenerator->getCollisionNum());
    DataWriter::writeBeamHoles(*this->major_compont_graph->beamLayoutConnectorGenerator, "init");


    glob_vars::config_file << "Beam Pre_computing + Initialization time: \t" << (double)(clock()-start)/CLOCKS_PER_SEC << endl;

}

void LEGOGenerator::stage_two_generation(){

    this->major_compont_graph->genComponentCandidates3D_weighted_picking_non_collision_withConnector();

    /*** filling gaps ***/
    for(auto& c : this->major_compont_graph->components)
        c->beamLayout->fillGap();

    this->major_compont_graph->tryTwoOrientForMinorComponents();

}

void LEGOGenerator::chooseModel(){
    printf("Please choose a input:\n");
    for(int i = 0; i<config::model_names.size(); i++){
        printf("%d \t %s\n",i,config::model_names.at(i).name.c_str());
    }
    int chosen_id = glob_vars::current_input;
    glob_vars::current_model = config::model_names.at(chosen_id);
    printf("current model:%s\n", glob_vars::current_model.name.c_str());
}

