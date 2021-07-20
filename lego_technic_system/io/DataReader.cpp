//
// Created by student on 4/25/2019.
//
#include <configure/global_variables.h>
#include "io/DataReader.h"
#include "bricks/BeamTemplate.h"
#include "util/util.h"
#include <fstream>


BeamTemplate *DataReader::getBeamByID(string Id) {
    assert(glob_vars::brickFactory != NULL);
    const auto &beam_list = glob_vars::brickFactory->getAllBeams();
    for (auto &b : beam_list) {
        if (Id.at(Id.length() - 1) == '\r')
            Id.erase(Id.length() - 1);
        if (b.ldraw_id == Id)
            return new BeamTemplate(b);
    }
    printf("No such beam in database: %s\n", Id.c_str());
    cin.get();
    return NULL;
}

Connector *DataReader::getConnectorByID(string Id) {
    const auto &connectors = glob_vars::brickFactory->getAllConnectors();
    for (auto c : connectors) {
        if (Id.at(Id.length() - 1) == '\r')
            Id.erase(Id.length() - 1);
        if (c.ldraw_id == Id)
            return new Connector(c);
    }
    printf("No such connector in database: %s\n", Id.c_str());
    cin.get();
    return NULL;
}

vector<BeamTemplate *> DataReader::readBeamsFromFile(string path) {
    using std::ifstream;
    vector<BeamTemplate *> result;
    vector<string> ss;
    ifstream fin(path);
    string line;
    while (getline(fin, line)) {
        line.erase(line.find_last_not_of(" \n\r\t") + 1);
        if (line.compare("") != 0 && line.at(0) == '1') {
            SplitString(line, ss, " ");
            BeamTemplate *b = getBeamByID(ss.at(14));
            if (b != NULL) {
                b->color = stoi(ss.at(1));
                b->translate(Vector3d(stod(ss.at(2)), stod(ss.at(3)), stod(ss.at(4))) / 20);
                Matrix3d trans_mat;
                trans_mat(0, 0) = stod(ss.at(5));
                trans_mat(0, 1) = stod(ss.at(6));
                trans_mat(0, 2) = stod(ss.at(7));
                trans_mat(1, 0) = stod(ss.at(8));
                trans_mat(1, 1) = stod(ss.at(9));
                trans_mat(1, 2) = stod(ss.at(10));
                trans_mat(2, 0) = stod(ss.at(11));
                trans_mat(2, 1) = stod(ss.at(12));
                trans_mat(2, 2) = stod(ss.at(13));
                b->transform(trans_mat);
                result.push_back(b);
            }
            ss.clear();
        }
    }

    return result;
}


vector<Connector *> DataReader::readConnectorsFromFile(string path) {
    using std::ifstream;
    vector<Connector *> result;
    vector<string> ss;
    ifstream fin(path);
    string line;
    while (getline(fin, line)) {
        if (line.compare("") != 0 && line.at(0) == '1') {
            SplitString(line, ss, " ");
            Connector *c = getConnectorByID(ss.at(14));
            if (c != NULL) {
                c->color = stoi(ss.at(1));
                c->translate(Vector3d(stod(ss.at(2)), stod(ss.at(3)), stod(ss.at(4))) / 20);
                Matrix3d trans_mat;
                trans_mat(0, 0) = stod(ss.at(5));
                trans_mat(0, 1) = stod(ss.at(6));
                trans_mat(0, 2) = stod(ss.at(7));
                trans_mat(1, 0) = stod(ss.at(8));
                trans_mat(1, 1) = stod(ss.at(9));
                trans_mat(1, 2) = stod(ss.at(10));
                trans_mat(2, 0) = stod(ss.at(11));
                trans_mat(2, 1) = stod(ss.at(12));
                trans_mat(2, 2) = stod(ss.at(13));
                c->transform(trans_mat);
                result.push_back(c);
            }
            ss.clear();
        }
    }
    return result;
}
