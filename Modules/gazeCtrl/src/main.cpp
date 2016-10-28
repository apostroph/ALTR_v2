/* 
 * Copyright (C): None
 * Authors: Jimmy Baraglia
 * Public License for more details
*/

#include "iCub/gaze.h"

int main(int argc, char * argv[]) {
    /* initialize yarp network */ 
    Network yarp;
    

    //system("mode con COLS=100");
    
    //test to launch a PDDL planner
    //system("cd ~/Robots/planner/problem/ && ../seq-sat-lama-2011/plan efaa-dom.pddl efaa-prob.pddl res_plan.txt") ;
    //system("cd ~/Robots/planner/problem/ && mv res_plan.txt* res_plan.txt") ;


    /* prepare and configure the resource finder */
    ResourceFinder rf;
    rf.setVerbose(true); 
    rf.setDefaultConfigFile("Gaze.ini"); //overridden by --from parameter
    rf.setDefaultContext("Gaze/conf");   //overridden by --context parameter
    rf.configure(argc, argv);
        /* create your module */
    gaze module(rf); 
    /* run the module: runModule() calls configure first and, if successful, it then runs */
    module.runModule(rf);

    return 0;
}

