/*  
 * Copyright (C): None
 * Authors: Jimmy Baraglia
 * Public License for more details
*/
#include <string>
#include <fstream>
#include <iostream>

#include <yarp/sig/all.h>
#include <yarp/dev/all.h>
#include <yarp/os/all.h>
#include <yarp/math/Math.h>

#include <unordered_map>

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;

namespace predictionUtils
{
    void saveData(string src, vector<string> states, vector<string> actions, vector< vector<string> > edges);
}