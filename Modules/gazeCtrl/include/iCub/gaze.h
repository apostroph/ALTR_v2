/* 
 * Copyright (C): None
 * Authors: Jimmy Baraglia
 * Public License for more details
*/

#ifndef _Gaze_H_
#define _Gaze_H_


#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/dev/all.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;

using namespace std;

	
enum colorSelection{red = 120, green = 55, blue = 10, orange = 110, purple = 165, yellow = 90};


class gaze : public RFModule {

public:
    /** 
     * document your methods too.
     */
    gaze(ResourceFinder &rf);
    ~gaze();

    bool configure(yarp::os::ResourceFinder &rf); // configure all the module parameters and return true if successful
    bool interruptModule();                       // interrupt, e.g., the ports 
    bool close();                                 // close and shut down the module
    bool respond(const yarp::os::Bottle& command, yarp::os::Bottle& reply);
    double getPeriod(); 
    bool updateModule();

private:
    Network yarp;
    string moduleName;
    
    string portOutName;
    yarp::os::Port portOut; // a port to receive information
    
    string portInName;
    yarp::os::Port portIn; // a port to receive information
    
    PolyDriver* robotHead;
    PolyDriver* robotHeadGaze;
    IGazeControl *head;
    
    bool finishedGaze;
    bool timeOut;
    double counter;
    
    
    double x, y, z;
    
    bool initPorts(yarp::os::ResourceFinder &rf);
    
    //Initialisation methods
    bool initRobotHead(yarp::os::ResourceFinder &rf);
    bool initGaze(yarp::os::ResourceFinder &rf);

    //Private method for head control
    void gazeTo(double x, double y, double z);
    
    void sendGazeDirection(bool block);
};


#endif // __Gaze_H__d

