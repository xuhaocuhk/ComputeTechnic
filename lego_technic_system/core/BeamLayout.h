//
// Created by 徐豪 on 2018/6/28.
//

#ifndef LEGO_TECHNIC_MAIN_BEAMLAYOUT_H
#define LEGO_TECHNIC_MAIN_BEAMLAYOUT_H


#include <vector>
#include <bricks/BrickCompare.h>
#include "bricks/BeamInstance.h"
#include "UnitEdge.h"
#include "UnitEdgeGroup.h"

using std::vector;
using std::map;

//TODO: Add connectors in the future
//表示一个beams的放置方式，同时代表着一个solution

struct OperationBeams{//在一个operation中发生变化的beams
    vector<BeamInstance*> oldBeams;
    vector<BeamInstance*> newBeams;
};

class Component;

class BeamLayout {
public:
    BeamLayout(vector<UnitEdge*>&, vector<LayerHole*>&, Component* );

    /**** SA Operations *****/
    void random_init();
    void random_init_symm();

    OperationBeams random_replace_symm();
    OperationBeams random_replace_symm_weighted_prob();
    OperationBeams random_replace_symm_weighted_prob_non_collision();
    OperationBeams random_replace_symm_weighted_prob_non_collision_connectness( UnitEdge* );
    OperationBeams random_replace_symm_connectivity( vector<UnitEdge*>& ); // operation to adjust connectivity

    ////Remove operators
    OperationBeams& eraseRandomSingleBeam  (OperationBeams &optBeams, vector<UnitEdge* > & ); // randomly erase one beam on a chosen edge
    OperationBeams& eraseBeamsOnEdgeAndHole(OperationBeams &optBeams, UnitEdge *chosenEdge);
    OperationBeams& eraseBeamsOnEdgeAndHole_affect(OperationBeams& optBeams);

    ////Information Update
    void updateOuterBeams(OperationBeams& optBeams);
    void updateSelfSymmetryInfo(OperationBeams& optBeams);

    ////Adding operators
    void pickRandomSingleBeam(OperationBeams & optBeams, vector<UnitEdge*>& consider_edges);
    void pickRandomBeams_allCovered(OperationBeams &optBeams);
    void pickRandomBeamsWithProbability_allCovered       (OperationBeams & optBeams); //pick beam according to hole covering number
    void pickRandomBeamsWithProbabiltiy_allCovered_affect(OperationBeams & optBeams); //pick beams without collision with outside components
    void pickRandomBeamsNoDominate_allCovered(OperationBeams &optBeams);

    void undoOperation(OperationBeams&);

    bool allEdgeCovered();
    bool allNonCollisionEdgesCovered();
    vector<UnitEdge*> unCoveredEdges();
    vector<UnitEdge*> unCoveredNonCollisionEdges();
    void addBeam(BeamInstance*);
    void eraseBeam(BeamInstance*);

    void chooseBestLayer(BeamInstance&);//返回true表示有更改，false表示没有更改
    //the second parameter is the lower and upper bound of the layout, will be updated inside the function
    std::pair<int, int> chooseBestLayer_insideBound(BeamInstance &, std::pair<int, int> &layer_bound);
    void chooseRandomLayer(BeamInstance&);
    void randomInitilizeLayers(vector<BeamInstance*>&);
    void randomInitilizeLayers(set<BeamInstance*, BeamInstance::BeamInstanceComp>&);

    void arrangeNoLayerBeamsLayer_Greedy (vector<BeamInstance *> &);

    void arrangeNoLayerBeamsLayer_Greedy(
            set<BeamInstance *, BeamInstance::BeamInstanceComp> &); //only arrange those beams without nominated layer number
    void arrangeNoLayerBeamsLayer_Greedy (OperationBeams&); //only do layer arranging for the beams just added

    bool isAdjToLayerBeam(BeamInstance &);//返回这个beam是不是没有layer并且与一个有layer的beam挨着
    bool isFloatingOnAir(BeamInstance &); //check whether the given beam is floating on the air

    set<UnitEdge*>      getAllRigidConnectedEdges(UnitEdge*);       //向外扩散返回所有的rigid邻接的edge，包括它自己
    set<BeamInstance*>  getAllRigidConnectedBeams(BeamInstance*);   //包括自己

    //get all adjacent rigidity connected beams of the current beam, not including self
    set<BeamInstance*> getAdjRigidConnectBeams(BeamInstance &);
    bool isRigidConnected(UnitEdge* u1, UnitEdge* u2);
    bool connectedOnHole(LayerHole*);
    int gapNumber(LayerHole*);
    static int gapNumber(vector<int>&);

    void fillGap(); //fill gaps for the current beams

    BeamLayout* copySelf();//deep copy a version of self

public:
    std::map<UnitEdge*,  set<BeamInstance*>> beamsOnEdge;
    std::map<LayerHole*, set<BeamInstance*>> beamsOnHole;
    std::map<LayerHole*, set<BeamInstance*>> outerBeamsOnHole; // beams outside this component occupies this hole
    std::map<LayerHole*, map<int, LayerHole*> > affected_hole; // mapping : (hole + layer) -> another hole on another component

    set<BeamInstance*, BeamInstance::BeamInstanceComp> beams;

    vector<UnitEdge* > unit_edges;
    vector<LayerHole*> holes;

    static const int MAX_LAYER = 5; //the max possible layer that will appear in this component
    Component* belonged_component = NULL;

};


#endif //LEGO_TECHNIC_MAIN_BEAMLAYOUT_H
