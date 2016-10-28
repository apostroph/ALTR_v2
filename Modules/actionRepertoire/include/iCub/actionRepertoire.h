/* 
 * Copyright: 2013 Robotcub Consortium
 * Author: Konstantinos Theofilis
 * Email: k.theofilis@herts.ac.uk
 * Copyright Policy: Released under the terms of the GNU GPL v2.0
 */

#include <string>
#include <cmath>

#include <iostream>
#include <fstream>

#include <yarp/os/RFModule.h>
#include <yarp/os/Network.h>
#include <yarp/sig/all.h>
#include <yarp/dev/all.h>
#include <yarp/os/all.h>
#include <yarp/math/Math.h>

#include <unordered_map>

#include "iCub/actionUtils.hpp"

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;

class actionRepertoire: public RFModule
{

public:
    
    double getPeriod();
    
    bool updateModule();

    bool respond(const Bottle& command, Bottle& reply);

    bool configure(yarp::os::ResourceFinder &rf);

    bool interruptModule();

    bool close();
    
private:   
    /* module parameters */
    string moduleName; //Name of the module
    string robotName; //Robot in use (iCub or icubSIM)
    
    Network *yarp;
    
    yarp::os::Port portAck; //Output port
    yarp::os::Port portInput; //Input port

    /* class parameters */
    bool rightArmEnabled;
    PolyDriver *clientCartCtrlRight;
    ICartesianControl *cartesianCtrlRight;
    yarp::sig::Vector dofRight;
     
    bool leftArmEnabled;
    PolyDriver *clientCartCtrlLeft;
    ICartesianControl *cartesianCtrlLeft;
    yarp::sig::Vector dofLeft;
    
    //Scene information
    yarp::sig::Vector limits; //[x_min, x_max, y_min, y_max, z_min, z_max]

    //Emotion variable
    int emotion_id; //0: neutral; 1: happy; 2: surprise; 3: fear; 4: disgust; 5: sadness; 6: anger;
    double trajectory_time;
    Vector motion_parameters; //Energy, attention, defense

    //Home position
    yarp::sig::Vector home_pose_right;
    yarp::sig::Vector home_rot_right;
    yarp::sig::Vector rest_right;
    
    yarp::sig::Vector home_pose_left;
    yarp::sig::Vector home_rot_left;
    yarp::sig::Vector rest_left;

    //Private methods
    bool openPorts(yarp::os::ResourceFinder &rf);
    bool openCartCon(string localName, string remoteName, PolyDriver **clientCartCtrl, ICartesianControl **cartesianCtrl, yarp::sig::Vector* dof);
    bool openControllers(yarp::os::ResourceFinder &rf);

    bool enableBody(ICartesianControl* cartesianCtrl, yarp::sig::Vector* dof);
    bool disableBody(ICartesianControl* cartesianCtrl, yarp::sig::Vector* dof);

    double set_traj_time(ICartesianControl* icart, double dist);

    void init_limits(yarp::os::ResourceFinder &rf);

    //Motion methods
    bool inRange(yarp::sig::Vector p1);
    void move_arm_to(ICartesianControl *icart, double x, double y, double z, double Rx, double Ry, double Rz);
    void execute_motion(ICartesianControl *icart, yarp::sig::Vector target, yarp::sig::Vector orientation);
    
    bool waitUntilTimeOut(double espected_duration, ICartesianControl* icart);
    
    bool push_to(yarp::sig::Vector object, yarp::sig::Vector dest, double strength = 1);
    bool wave(bool right_Nleft, float intensity = 0.5, int nb_wave = 2);
    bool touch(yarp::sig::Vector dest);

    void go_home(ICartesianControl *ctrl, Vector home_pos, Vector home_rot);
    
    void acknowledge(string message);

};
