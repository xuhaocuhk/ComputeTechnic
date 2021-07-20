//
// Created by 徐豪 on 2018/5/19.
//

#ifndef LEGOGENERATION_MAIN_DATAWRITER_H
#define LEGOGENERATION_MAIN_DATAWRITER_H


#include "bricks/BeamTemplate.h"
#include "bricks/BeamInstance.h"
#include "connector/BeamLayoutConnectorGenerator.h"

using std::ofstream;

class ConstructionGraph;
class BeamLayoutConnectorGenerator;

class DataWriter {

public:
    /**** Statistics output ****/
    static void initDebugDirectory();
    static void writeProgramStatistics(ofstream &fout);
    static void writeInputStatistics(ofstream & fout);

    static void writeDecomposeSAStatistics(string content, string remark);

    static void writeGenerationSAStatistics(string content, string remark);

    static void
    writeComponentGenerationParameters(int comp_id, double time_limit, double ka_min, double ka_max, double T_min,
                                       double gen_time, string gen_result);

    /**** Intermediate results in the paper *****/
    //whether write in local "python" directory
    static void writeUnitEdges(const set<UnitEdge*, UnitEdge::UnitEdgeIDComp> &, const set<LayerHole*, LayerHole::LayerHoleComp>&, string remark, bool write_local = false);
    static void writeUnCoveredUnitEdge( string remark );
    static void writeHoleOrientation(vector<SketchLine *> &, vector<SketchPoint *> &, string remark);

    static void writeGroupingResult_debug(vector<LayerGraph *> &, string remark);

    static void writeBeamInstances(set<BeamInstance*, BeamInstance::BeamInstanceComp>&, string remark);
    static void writeBeamInstances(ConstructionGraph &, string remark);
    static void writeBeamInstances(ConstructionGraph &, BeamLayoutConnectorGenerator& beamGenerator, string remark, bool indiv_conn = false, bool symm_conn = false);
    static void writeBeamHoles(BeamLayoutConnectorGenerator& beamGenerator, string remark);
    static void writeFinalResultStatistics(ConstructionGraph &, BeamLayoutConnectorGenerator& beamGenerator);
    static void writeFinalResultComplexity(ConstructionGraph &, BeamLayoutConnectorGenerator& beamGenerator);


    static ofstream getOutputFileStream(string file_path);

    /**** For output the illustration data ****/
    static void writeAllBricks();
};


#endif //LEGOGENERATION_MAIN_DATAWRITER_H
