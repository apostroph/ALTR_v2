/* Copyright: 2013 Robotcub Consortium
 * Author: Konstantinos Theofilis, University of Hertfordshire
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */

#include <yarp/os/Network.h>
#include <yarp/os/Time.h>
#include <string>
#include <iostream>
#include <stdio.h>

#include "iCub/Recognition.h"

using namespace std;
using namespace yarp::os;

int main (int argc, char *argv[]) {
    Network yarp;
    

    Recognition mod;
    ResourceFinder rf;
    rf.setVerbose();
    rf.setDefaultConfigFile("config.ini");
    rf.setDefaultContext("template");
    rf.configure(argc, argv);
    if (!mod.configure(rf)) {
        cout << "Configuration failed" << endl;
        return 0;
    }

    mod.runModule();
}
