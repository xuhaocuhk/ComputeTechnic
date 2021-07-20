//
// Created by 徐豪 on 2018/5/19.
//

#include <fstream>
#include "ColorTable.h"
#include "DataWriter.h"
#include "configure/global_variables.h"
#include "util/util.h"
#include "util/Helper.h"
//#include <json.h>
#include <dirent.h>
#include <bricks/BrickFactory.h>
#include <iomanip>      // std::setw
#include "util/Symmetry.h"
#if defined(_WIN32) || defined(WIN32) || defined(_WIN64) || defined(WIN64)
#include <dir.h>
#include <util/Symmetry.h>

#endif


using std::endl;
using std::cout;
using std::ofstream;
using std::to_string;
using std::setw;

void DataWriter::initDebugDirectory(){
    printf("Initialize debugging directory...\n");

    char buff[100];   snprintf(buff, sizeof(buff), "%s_%s", glob_vars::current_model.name.c_str(), getCurrentTimeAsString().c_str());
    std::string dir_name = buff;
    glob_vars::current_debug_dir = glob_vars::PRE_FIX + "1_debug/"+ dir_name;
    cout<<"current debug directory: "<<dir_name<<endl;
    string mkdir_command = "mkdir \"" + glob_vars::PRE_FIX + "1_debug/"+dir_name + "\"";

    system(mkdir_command.c_str());

    glob_vars::config_file = DataWriter::getOutputFileStream(glob_vars::current_debug_dir + "/global_configure.txt");
}


void DataWriter::writeBeamInstances(set<BeamInstance*, BeamInstance::BeamInstanceComp>& beams, string remark){
    string file_path = glob_vars::current_debug_dir + "/" +  glob_vars::current_model.name + "_" + "beams" + remark +  ".ldr";
    using std::ofstream;
    ofstream fout;
    fout.open(file_path);
    for(auto& b : beams){
        if(b->layer_number != NO_LAYER)
            fout<<b->toLdrawFormatAsString();
    }
    fout.close();
    printf("File %s created\n",file_path.c_str());
}

void DataWriter::writeBeamInstances(ConstructionGraph& consGraph, string remark){
    string file_path = glob_vars::current_debug_dir + "/" +  glob_vars::current_model.name + "_" + "beams" + remark +  ".ldr";
    using std::ofstream;
    ofstream fout;
    fout.open(file_path);
    for(auto& comp : consGraph.components){
        vector<BeamTemplate *> m_beams;

        for(auto& b : comp->beamLayout->beams) m_beams.push_back(b->getBeam());

        if(comp->layerGraph->sketchGroup->isSelfSymmetry()){

            //TODO : consider the influence of adjacent holes
            for(auto& bi : comp->beamLayout->beams){
                if (bi->layer_number != 0) {
                    bool collision = false;

                    for (auto &h : bi->template_beam->covered_holes) {
                        if (std::find_if(comp->beamLayout->beamsOnHole[h].begin(),
                                         comp->beamLayout->beamsOnHole[h].end(),
                                         [bi](BeamInstance *beamInstance) {
                                             return beamInstance->layer_number == -bi->layer_number;
                                         })
                            != comp->beamLayout->beamsOnHole[h].end()) {
                            collision = true;
                            break;
                        }
                    }

                    if (!collision) {
                        BeamTemplate *nb = bi->getBeam();
                        nb->layer = -nb->layer;
                        m_beams.push_back(nb);
                    }
                }
            }
        }

        if(comp->layerGraph->containSymmetryComponentsAllDirection()){
            ////beams from symmetry components
            vector<BeamTemplate *> symm_beams = Symmetry::getSymmetryComponentsAsBeams(m_beams,
                                                                                       comp->layerGraph->getCurrentPlaneFlag(),
                                                                                       comp->layerGraph->sketchGroup->getAllSymmetryGroupsWithDirecTag());
            for(auto& b : symm_beams) b->color = 4;

            m_beams.insert(m_beams.end(),symm_beams.begin(),symm_beams.end());

        }

        for(auto& b : m_beams){
            fout<<b->toLDrawFormatAsString();
        }

    }
    fout.close();
    printf("File %s created\n",file_path.c_str());
}

void DataWriter::writeBeamInstances(ConstructionGraph & consGraph, BeamLayoutConnectorGenerator& beamGenerator, string remark, bool indiv_conn, bool symm_conn){
    string file_path = glob_vars::current_debug_dir + "/" +  glob_vars::current_model.name + "_" + "beams" + remark +  ".ldr";
    using std::ofstream;
    ofstream fout;
    fout.open(file_path);
    for(auto& comp : consGraph.components){
        vector<BeamTemplate *> m_beams;

        for(auto& b : comp->beamLayout->beams) m_beams.push_back(b->getBeam());

        if(comp->layerGraph->sketchGroup->isSelfSymmetry()){

            //TODO : consider the influence of adjacent holes
            for(auto& bi : comp->beamLayout->beams){
                if (bi->layer_number != 0) {
                    bool collision = false;

                    for (auto &h : bi->template_beam->covered_holes) {
                        if (std::find_if(comp->beamLayout->beamsOnHole[h].begin(),
                                         comp->beamLayout->beamsOnHole[h].end(),
                                         [bi](BeamInstance *beamInstance) {
                                             return beamInstance->layer_number == -bi->layer_number;
                                         })
                            != comp->beamLayout->beamsOnHole[h].end()) {
                            collision = true;
                            break;
                        }
                    }

                    if (!collision) {
                        BeamTemplate *nb = bi->getBeam();
                        nb->layer = -nb->layer;
                        m_beams.push_back(nb);
                    }
                }
            }
        }

        if(comp->layerGraph->containSymmetryComponentsAllDirection()){
            ////beams from symmetry components
            vector<BeamTemplate *> symm_beams = Symmetry::getSymmetryComponentsAsBeams(m_beams,
                                                                                       comp->layerGraph->getCurrentPlaneFlag(),
                                                                                       comp->layerGraph->sketchGroup->getAllSymmetryGroupsWithDirecTag());
            for(auto& b : symm_beams) b->color = 4;

            m_beams.insert(m_beams.end(),symm_beams.begin(),symm_beams.end());

        }

        for(auto& b : m_beams){
            fout<<b->toLDrawFormatAsString();
        }
    }

    if(!symm_conn){
        for(auto& c : beamGenerator.connectors){
            if(indiv_conn){
                fout<<c->toIndividualLdrawAsString();
            }else{
                fout<<c->toLDrawFormatAsString();
            }
        }
    }else{
        ////...output original connectors and symmetry connectors
        set<UnitEdgeGroup*, UnitEdgeGroup::UnitEdgeGroupComp> ueGroups = set<UnitEdgeGroup*, UnitEdgeGroup::UnitEdgeGroupComp>(
                beamGenerator.uncoverd_uedges_groups.begin(), beamGenerator.uncoverd_uedges_groups.end() );
        vector<Connector*> total_connectors;
        while( !ueGroups.empty() ){
            UnitEdgeGroup* group = (*ueGroups.begin());
            auto symm_groups = group->getAllSymmetryUnitEdgeGroupsWithDirecTag();

            ueGroups.erase(group);
            for(auto& g : symm_groups)
                ueGroups.erase( std::get<0>(g) );

            auto symm_conns = group->getSymmetryConnectors( group->assigned_connectors );
            total_connectors.insert(total_connectors.end(), group->assigned_connectors.begin(), group->assigned_connectors.end());
            total_connectors.insert(total_connectors.end(), symm_conns.begin(), symm_conns.end());
        }
        for(auto& c : total_connectors){
            if(indiv_conn){
                fout<<c->toIndividualLdrawAsString();
            }else{
                fout<<c->toLDrawFormatAsString();
            }
        }
        for(auto& c : beamGenerator.straight_conns){
            if(indiv_conn){
                fout<<c->toIndividualLdrawAsString();
            }else{
                fout<<c->toLDrawFormatAsString();
            }
        }
    }

    fout.close();
    printf("File %s created\n",file_path.c_str());
}

void DataWriter::writeBeamHoles(BeamLayoutConnectorGenerator& beamGenerator, string remark){
    string file_path = glob_vars::current_debug_dir + "/" +  glob_vars::current_model.name + "_" + "beamHoles" + remark +  ".ldr";
    using std::ofstream;
    ofstream fout;
    fout.open(file_path);

    for(auto& bh : beamGenerator.beamholes){
        fout<<bh->toLDrawFormatAsString();
    }

    fout.close();
    printf("File %s created\n",file_path.c_str());
}

void DataWriter::writeFinalResultStatistics(ConstructionGraph & constructionGraph, BeamLayoutConnectorGenerator& beamGenerator){
    glob_vars::config_file << "\n\n\n-----------------Final Result Quality ------------------\n";

    /* extremly simplicity settings */
    auto weight = glob_vars::current_model.generating_weight;
    ////// dev                simp                 rigid
    double w_dev = weight[0], w_simp = weight[1], w_rigid = weight[2] ;
    ////// gap                bound                symm                  collision            connection
    double w_gap = weight[3], w_bound = weight[4], w_lsymm = weight[5],  w_colld = weight[6], w_conn = weight[7], w_phrat = weight[8] ;

    double deviation_score    = 0.0;
    double simplicity_score   = 0.0;
    double rigidity_score     = 0.0;
    double gaps_score         = 0.0;
    double layerbound_score   = 0.0;
    double layer_symm         = 0.0;
    double collision_score    = 0.0; //TODO: evaluate collision 的时候symmetry 也有问题
    double connection_score   = 0.0;
    double pin_head_ratio     = 0.0;

    /**** Deviation ****/
    deviation_score           = Grader::deviation2sketch_online_3D(constructionGraph);
//    double dev_validate = Grader::deviation2sketch_3D(constructionGraph);
//    printf("deviation:(full, local) (%lf, %lf)\n", dev_validate, deviation_score);
//    assert(dev_validate == deviation_score);

    /**** Simplicity ****/
    simplicity_score          = Grader::simplicity_online_3D(constructionGraph);
//    double simp_validate = Grader::simplicity_3D(constructionGraph);
//    printf("simplicity:(full, local) (%lf, %lf)\n", simp_validate, simplicity_score);
//    assert(simp_validate == simplicity_score);

    /**** Rigidity ****/
    rigidity_score             = Grader::rigidity_transitive_online_3D(constructionGraph);
//    double rigidity_validate = Grader::rigidity_transitive_3D(constructionGraph);
//    printf("rigidity: (full, local) (%lf, %lf)\n", rigidity_validate, rigidity_score);
//    assert(rigidity_validate == rigidity_score );

    /**** Gaps ****/
    gaps_score                 = Grader::gaps_online_3D(constructionGraph);
//    double gap_validate = Grader::gaps_3D(constructionGraph);
//    printf("gaps: (full, local) (%lf, %lf)\n", gap_validate, gaps_score);
//    assert(gap_validate == gaps_score);

    /**** Layer Bound ****/
    layerbound_score           = Grader::layer_bound_online_3D(constructionGraph);
//    double layerbound_validate = Grader::layer_bound_3D(constructionGraph);
//    printf("layerbound: (full, local) (%lf, %lf)\n", layerbound_validate, layerbound_score);
//    assert(layerbound_validate == layerbound_score);

    /**** Layer Symmetry ****/
    layer_symm                 = Grader::layer_symmetry_online_3D(constructionGraph);
//    double layer_symm_validate = Grader::layer_symmetry_3D(constructionGraph);
//    printf("layer_symm: (full, local) (%lf, %lf)\n", layer_symm_validate, layer_symm);
//    assert(layer_symm_validate == layer_symm);

    /**** Collision ****/
    collision_score  = Grader::collision_num_3D(constructionGraph);
//    printf("collison:%lf\n", collision_score);

    /**** Number of unconnected groups ****/
    connection_score = Grader::connection_score_3D(constructionGraph);

    /**** Non Pin Head Ratio ****/
    vector<Connector*> connectors = vector<Connector*>(beamGenerator.connectors.begin(), beamGenerator.connectors.end());
    pin_head_ratio = Grader::Nonpin_head_ratio(connectors);

    double total_score      = w_dev   * deviation_score +
                              w_simp  * simplicity_score +
                              w_rigid * rigidity_score +
                              w_gap   * gaps_score +
                              w_bound * layerbound_score * layerbound_score +
                              w_lsymm * layer_symm +
                              w_colld * collision_score +
                              w_conn  * connection_score +
                              w_phrat * pin_head_ratio;

    glob_vars::config_file<<"deviation:  \t" << setw(5) << w_dev  << "\t*\t" << setw(8) << deviation_score                      << "\t=\t" << w_dev * deviation_score   << endl;
    glob_vars::config_file<<"simplicity: \t" << setw(5) <<w_simp  << "\t*\t" << setw(8) << simplicity_score                     << "\t=\t" << w_simp * simplicity_score  << endl;
    glob_vars::config_file<<"rigidity:   \t" << setw(5) <<w_rigid << "\t*\t" << setw(8) << rigidity_score                       << "\t=\t" << w_rigid * rigidity_score   << endl;
    glob_vars::config_file<<"gaps:       \t" << setw(5) <<w_gap   << "\t*\t" << setw(8) << gaps_score                           << "\t=\t" << w_gap * gaps_score << endl;
    glob_vars::config_file<<"bound:      \t" << setw(5) <<w_bound << "\t*\t" << setw(8) << layerbound_score * layerbound_score  << "\t=\t" << w_bound * layerbound_score * layerbound_score<< endl;
    glob_vars::config_file<<"layer_symm: \t" << setw(5) <<w_lsymm << "\t*\t" << setw(8) << layer_symm                           << "\t=\t" << w_lsymm * layer_symm << endl;
    glob_vars::config_file<<"collision:  \t" << setw(5) <<w_colld << "\t*\t" << setw(8) << collision_score                      << "\t=\t" << w_colld * collision_score << endl;
    glob_vars::config_file<<"connection: \t" << setw(5) <<w_conn  << "\t*\t" << setw(8) << connection_score                     << "\t=\t" << w_conn * connection_score << endl;
    glob_vars::config_file<<"phead_ratio:\t" << setw(5) <<w_phrat << "\t*\t" << setw(8) << pin_head_ratio                       << "\t=\t" << w_phrat * pin_head_ratio << endl;
    glob_vars::config_file<<"total: "<< total_score << endl;

}

void DataWriter::writeFinalResultComplexity(ConstructionGraph & constructionGraph, BeamLayoutConnectorGenerator& beamGenerator){

    int beam_brick_num = 0, connector_num = 0;
    int beam_num = 0; int total_beam_length = 0;

    for(auto& comp : constructionGraph.components){
        vector<BeamTemplate *> m_beams;

        for(auto& b : comp->beamLayout->beams) m_beams.push_back(b->getBeam());

        if(comp->layerGraph->containSymmetryComponentsAllDirection()){
            ////beams from symmetry components
            vector<BeamTemplate *> symm_beams = Symmetry::getSymmetryComponentsAsBeams(m_beams,
                                                                                       comp->layerGraph->getCurrentPlaneFlag(),
                                                                                       comp->layerGraph->sketchGroup->getAllSymmetryGroupsWithDirecTag());
            m_beams.insert(m_beams.end(),symm_beams.begin(),symm_beams.end());

        }else if(comp->layerGraph->sketchGroup->isSelfSymmetry()){

            //TODO : consider the influence of adjacent holes
            for(auto& bi : comp->beamLayout->beams){
                if (bi->layer_number != 0) {
                    bool collision = false;
                    for (auto &h : bi->template_beam->covered_holes) {
                        if (std::find_if(comp->beamLayout->beamsOnHole[h].begin(),
                                         comp->beamLayout->beamsOnHole[h].end(),
                                         [bi](BeamInstance *beamInstance) {
                                             return beamInstance->layer_number == -bi->layer_number;
                                         })
                            != comp->beamLayout->beamsOnHole[h].end()) {
                            collision = true;
                            break;
                        }
                    }

                    if (!collision) {
                        BeamTemplate *nb = bi->getBeam();
                        nb->layer = -nb->layer;
                        m_beams.push_back(nb);
                    }
                }
            }
        }

        for(auto& b : m_beams){
            beam_num++;
            total_beam_length += b->holeNum();
            beam_brick_num += (b->sub_beams.empty()? 1 : b->sub_beams.size());
        }
    }

    for(auto& c : beamGenerator.connectors){
        connector_num += ( c->sub_bricks.empty() ? 1 : c->sub_bricks.size() );
    }

    glob_vars::config_file<<"Total Beam Brick Num: "       <<  beam_brick_num << endl;
    glob_vars::config_file<<"Total Connector Num: "  << connector_num << endl;
    glob_vars::config_file<<"Total Beam Num: "       <<  beam_num << endl;
    glob_vars::config_file<<"Total Beam length: "  << total_beam_length << endl;
}

ofstream DataWriter::getOutputFileStream(string file_path){
    ofstream fout;
    fout.open(file_path);
    return fout;
}

void DataWriter::writeProgramStatistics(ofstream &fout){
    fout<< "# This file is auto-generated from XH's program \n";

    fout<<"current model: " << glob_vars::current_model.name <<"\n";
    fout<<"current sketch: "<< glob_vars::current_model.sketch_path <<"\n";
    fout<<"current rand seed: " << glob_vars::current_model.seed_number<<"\n";
    fout<<"generation weight:\n";
    auto& w = glob_vars::current_model.generating_weight;
    fout<<"w_dev: " << w[0] <<
    " w_simp: "  << w[1] <<
    " w_rigid: " << w[2] <<
    " w_gap: "   << w[3] <<
    " w_bound: " << w[4] <<
    " w_lsymm: " << w[5] <<
    " w_colld: " << w[6] <<
    " w_conn:  " << w[7] << endl;

    fout.flush();
}

void DataWriter::writeInputStatistics(ofstream & fout){
    fout << "------------- Input Statistics ----------------" << endl;
    fout<<"#sketch lines: \t"        << glob_vars::sketchGraph->m_lines->size()  << endl;

    fout<<"#sketch points: \t"        << glob_vars::sketchGraph->m_points->size() << endl;

    fout<<"#holes: \t"        << glob_vars::guidingGraph->m_holes.size()   << endl;

    int total_length = 0;
    for(auto& l : *glob_vars::sketchGraph->m_lines) total_length += l->intLength();
    fout<<"total length: \t " << total_length << endl;

//    fout<<"symm_x:\t"<<glob_vars::current_model.symmetry_x <<"\tsymm_y:\t"<<glob_vars::current_model.symmetry_y <<"\tsymm_z:\t"<<glob_vars::current_model.symmetry_z << "\n\n\n";
//
//    Write physical dimension
//    double max_x = -99999, max_y = -99999, max_z = -99999;
//    double min_x = 99999, min_y = 99999, min_z = 99999;
//    for(auto& p : *glob_vars::sketchGraph->m_points){
//        if (p->_x > max_x) max_x = p->_x;
//        if (p->_y > max_y) max_y = p->_y;
//        if (p->_z > max_z) max_z = p->_z;
//        if (p->_x < min_x) min_x = p->_x;
//        if (p->_y < min_y) min_y = p->_y;
//        if (p->_z < min_z) min_z = p->_z;
//    }
//    printf("\n model name : %s\n", glob_vars::current_model.name.c_str());
//    printf("X:%lf\n", max_x - min_x);
//    printf("Y:%lf\n", max_y - min_y);
//    printf("Z:%lf\n", max_z - min_z);
//    cin.get();

}


void DataWriter::writeUnitEdges(const set<UnitEdge*, UnitEdge::UnitEdgeIDComp >& u_edges,
                                const set<LayerHole*, LayerHole::LayerHoleComp>& u_holes,
                                string remark, bool write_local){
    string file_path = (write_local ? "../python" : glob_vars::current_debug_dir) +
                        "/" + glob_vars::current_model.name + "_unitEdges_" + remark + ".txt";
    ofstream fout = DataWriter::getOutputFileStream(file_path);

    for(auto& h : u_holes){
        fout<<"v "<<h->_id << " "<<h->_x<<" "<<h->_y<<" "<<h->_z<<endl;
    }
    for(auto& ue : u_edges){
        fout<<"l "<< ue->_id <<" "<<ue->hole1->_id<<" "<<ue->hole2->_id<<endl;
    }
    printf("File %s created\n",file_path.c_str());
    fout.close();
}

void DataWriter::writeUnCoveredUnitEdge( string remark ){

    set<UnitEdge*, UnitEdge::UnitEdgeIDComp> uncovered_edges;
    for(auto& ue : glob_vars::guidingGraph->u_edges){
        auto symm_uedges = ue->getAllSymmetryUnitEdges();

        auto iter = std::find_if(symm_uedges.begin(), symm_uedges.end(), [](UnitEdge* symm_ue){
            return symm_ue->belonged_beamLayout != NULL && !symm_ue->belonged_beamLayout->beamsOnEdge[symm_ue].empty();
        });

        if( iter == symm_uedges.end() ){ //all symmetry edges are not covered
            uncovered_edges.insert(ue);
        }
    }

    DataWriter::writeUnitEdges( uncovered_edges,
                                glob_vars::guidingGraph->m_holes,
                                remark, true);
}

void DataWriter::writeHoleOrientation(vector<SketchLine *> &m_lines, vector<SketchPoint *> &m_points, string remark){
    string file_path = glob_vars::current_debug_dir + "/" + glob_vars::current_model.name + "_Orientation_SA_" + remark + ".ldr";
    ofstream fout = DataWriter::getOutputFileStream(file_path);
    //print all lines
    for(auto& l : m_lines){
        auto l_holes = l->getInternalHoles();
        for(auto& h : l_holes){
            int color = 4;
            switch (l->chosen_orientation){
                case Constants::PLANE_XY:{
                    color = ColorTable::BLUE;//blue
                    break;
                }
                case Constants::PLANE_XZ:{
                    color = ColorTable::GREEN; //green
                    break;
                }
                case Constants::PLANE_YZ:{
                    color = ColorTable::RED;//red
                    break;
                }
                default:{
                    printf("Non orthorgnal direction hole!!!\n");
                }
            }
            fout<<h->toLdrawFormatAsString(color);
        }
//            fout<<h->toLdrawFormatAsString(10);
    }
    //print intersection holes
    for(auto& p : m_points){
        int orthrg_num = 0;
        auto adj_lines = p->getAdjLines();
        for(auto& l : adj_lines){
            if (l->chosen_orientation != p->chosen_orientation){
                orthrg_num ++;
            }
        }
        int color = 4;
        switch (p->chosen_orientation){
            case Constants::PLANE_XY:{
                color = ColorTable::BLUE;//blue
                break;
            }
            case Constants::PLANE_XZ:{
                color = ColorTable::GREEN; //green
                break;
            }
            case Constants::PLANE_YZ:{
                color = ColorTable::RED;//red
                break;
            }
            default:{
                printf("Non orthorgnal direction hole!!!\n");
            }
        }
        fout<<p->getHole()->toLdrawFormatAsString(color);
//        fout<<p->getHole()->toLdrawFormatAsString(orthrg_num == 0? 10 : 3+orthrg_num);
    }
    printf("File %s created\n",file_path.c_str());
    fout.close();
}

void DataWriter::writeDecomposeSAStatistics(string content, string remark){
    string file_path = glob_vars::current_debug_dir + "/" +  "SA_obj_value_statistics_" + remark +  ".txt";
    using std::ofstream;
    ofstream fout;
    fout.open(file_path);
    fout<<content<<endl;
    fout.close();
    printf("File %s created\n",file_path.c_str());
}

void DataWriter::writeComponentGenerationParameters(int comp_id, double time_limit, double ka_min, double ka_max, double T_min,
                                                    double gen_time, string gen_result){
//    Json::Value root;
//    root["component_id"] = comp_id;
//    root["time_limit"] = time_limit;
//    root["ka_min"] = ka_min;
//    root["ka_max"] = ka_max;
//    root["T_min"] = T_min;
//    root["generation_time"] = gen_time;
//    root["result_quality"] = gen_result;//the gen_result include objective function value and weight
//    auto str = root.toStyledString();
//    cout<<str<<endl;
//    string file_path = glob_vars::current_debug_dir + "/" +  "component_" +  std::to_string(comp_id) + "_config.json";
//    std::ofstream fout;
//    fout.open(file_path);
//    fout<<str;
//    fout.close();
//    printf("File %s created\n",file_path.c_str());
}

void DataWriter::writeGenerationSAStatistics(string content, string remark){
    string file_path = glob_vars::current_debug_dir + "/" + "generationSAStatistics" + remark +  ".txt";
    using std::ofstream;
    ofstream fout;
    fout.open(file_path);
    fout<<content<<endl;
    fout.close();
    printf("File %s created\n",file_path.c_str());
}

void DataWriter::writeGroupingResult_debug(vector<LayerGraph *> &layers, string remark){
    string file_path = glob_vars::current_debug_dir + "/" + "grouping" + ".ldr";
    using std::ofstream;
    ofstream fout;
    fout.open(file_path);

    int color = 12;
    for(auto& layer : layers){
        fout<<layer->getCurrentHolesAsString(color++);
    }
    fout.close();
    printf("File %s created\n",file_path.c_str());
}

void DataWriter::writeAllBricks(){
    BrickFactory brickFactory;
    auto all_beams = brickFactory.getAllBeams();
    auto all_conns = brickFactory.getAllConnectors();

    for(auto& b : all_beams){
        string file_path = glob_vars::current_debug_dir + "/" + b.ldraw_id + ".ldr";
        using std::ofstream;
        ofstream fout;
        fout.open(file_path);
        fout<<b.toLDrawFormatAsString(15);
        fout.close();
    }

    for(auto& c : all_conns){
        string file_path = glob_vars::current_debug_dir + "/" + c.ldraw_id + ".ldr";
        using std::ofstream;
        ofstream fout;
        fout.open(file_path);
        fout<<c.toLDrawFormatAsString();
        fout.close();
    }
}