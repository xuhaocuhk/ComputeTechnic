//
// Created by 徐豪 on 2018/5/5.
//

#include <dirent.h>
#include "bricks/BrickFactory.h"
#include "objectives/Grader.h"
#include <fstream>
#include <io/DataReader.h>
#include "configure/global_variables.h"
#include "PostProcessing.h"

void PostProcessing::processSymmetryConnector(){
    double ref_plane_y = 0.0;
    int file_id = 0;
    DIR *pDIR;
    struct dirent *entry;
    string dir_path = glob_vars::PRE_FIX + glob_vars::current_model.getDirectoryName() + "/generate/2_pre_final/";
    if(pDIR=opendir(dir_path.c_str())){
        while(entry = readdir(pDIR)){
            if( strcmp(entry->d_name, ".") != 0 && strcmp(entry->d_name, "..") != 0 )
                if(strstr(entry->d_name,".ldr")!=NULL){
                    std::cout<<"loading file "<<entry->d_name<<endl;
                    BrickFactory brickFactory;
                    auto connectors = DataReader::readConnectorsFromFile(dir_path + entry->d_name);
                    auto beams = DataReader::readBeamsFromFile(dir_path + entry->d_name);
                    assert(!connectors.empty());
                    vector<Connector*> lower_side_conns;
                    vector<Connector*> cross_conns;
                    for(auto& c: connectors){
                        if(c->isLowerSide(Constants::PLANE_XZ,0.0)){
                            lower_side_conns.push_back(c);
                        }
                        if(c->isCrossPlane(Constants::PLANE_XZ,0.0))
                            cross_conns.push_back(c);
                    }
                    vector<Connector*> ref_conns;
                    for(auto& c : lower_side_conns){
                        Connector* conn = new Connector(*c);
                        conn->translation.y() *= -1.0;
                        Matrix3d ref_mat = Matrix3d::Identity(3,3);
                        ref_mat(1,1) = -1;
                        conn->transform(ref_mat);
                        ref_conns.push_back(conn);
                    }
                    string file_path = glob_vars::PRE_FIX + glob_vars::current_model.getDirectoryName() + "/generate/2_pre_final/symm_" + entry->d_name;
                    using std::ofstream;
                    ofstream fout;
                    fout.open(file_path);

                    for(auto& b : beams)
                        fout<<b->toLDrawFormatAsString();
                    for(auto& c : cross_conns)
                        fout<<c->toLDrawFormatAsString();
                    for(auto& c : lower_side_conns)
                        fout<<c->toLDrawFormatAsString();
                    for(auto& c : ref_conns)
                        fout<<c->toLDrawFormatAsString();
                    fout.close();
                    cin.get();
                }
        }
        closedir(pDIR);
    }
}