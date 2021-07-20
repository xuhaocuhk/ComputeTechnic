//
// Created by 徐豪 on 2018/4/28.
//

#include "configure/global_variables.h"

bool inBoundingBox(vector<BeamTemplate *> &beams) {
    if(glob_vars::current_model.name == "loomachine_middle"){
        double x_min = -12.5;
        double x_max = 7.5;
        double z_min = -2.5;
        double z_max = 6.5;

        for(auto& b : beams){
            for(auto& h : b->getCurrentComponentHoles()){
                if(h.x()<3.5 && h.x()>-1.5){//中间的两根
                    if((h.x()>-1.5&&h.x()<-0.5) || (h.x()>0.5&&h.x()<1.5) || (h.x()>2.5&&h.x()<3.5))
                        return false;
                    if(h.z()>13.5)
                        return false;
                    if(h.x()>-0.3 && h.x()<0.3 && h.z()>12.5)
                        return false;
                }else{
                    if(h.x()<x_min || h.x()> x_max || h.z()<z_min || h.z()>z_max)
                        return false;
                }
            }
        }
        return true;
    }else if(glob_vars::current_model.name == "complexframe"){
        return true;
    }else if(glob_vars::current_model.name == "3dprinter"){
        return true;
    }
    printf("No difinition of such bounding box yet!!!\n");
    exit(1);
}

bool inBoundingBox(vector <Connector*> & connectors){
    if(glob_vars::current_model.name == "3dprinter"){
        for(auto& c : connectors){
            for(auto& pin : c->getCurrentNonInsertablePins()){
                if(pin.pos.x()>2.12 || pin.pos.x()<-16.0)
                    return false;
            }
        }
        return true;
    }
    return true;
}

bool inBoundingBox(const Connector& connector){
    if(glob_vars::current_model.name == "3dprinter"){
//        for(auto& pin : connector.getCurrentNonInsertablePins()){
//            if(pin.pos.x()>2.12 || pin.pos.x()<-16.0)
//                return false;
//        }
        return true;
    }else if(glob_vars::current_model.name == "lifter"){
        printf("Processing bounding box for lifter!!\n");

        for(auto& pin : connector.getCurrentNonInsertablePins()){
            if(pin.pos.z()>12.05 || pin.pos.z()< -12.05 )
                return false;
        }
        return true;

    }
    return true;
}