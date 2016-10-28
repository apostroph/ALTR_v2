/* 
 * Copyright (C): None
 * Authors: Jimmy Baraglia
 * Public License for more details
*/
#include <string>

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

namespace actionUtils
{
    void set_motion_parameters(Vector& motion_parameters, int emotion_id);
    
    /**
    * Give the angle between two points (2D)
    */
    float get_angle_of_line_between_two_points(yarp::sig::Vector p1, yarp::sig::Vector p2);
    
    void send_gaze_target(yarp::os::Port* port, yarp::sig::Vector gaze_target);
    
    void get_motion_orientation(double Rx, double Ry, double Rz, yarp::sig::Vector* orientation); 
}