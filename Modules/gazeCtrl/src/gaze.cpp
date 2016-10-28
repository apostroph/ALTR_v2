/* 
 * Copyright (C): None
 * Authors: Jimmy Baraglia
 * Public License for more details
*/

#include "iCub/gaze.h"


gaze::gaze(ResourceFinder &rf):
x(-0.4), y(0), z(0)
{
}

gaze::~gaze()
{
}


bool gaze::configure(yarp::os::ResourceFinder &rf) {        
	bool bEveryThingisGood = true;
        bool tracking, saccadic, stabilization;
	//init the network
	Network::init();
	
	moduleName = rf.check("name", Value("gazeController"), "module name (string)").asString();
        
	tracking = rf.check("tracking", Value(false), "Tracking mode on/off").asBool();
        
	saccadic = rf.check("saccadic", Value(false), "Saccadic mode on/off").asBool();
        
	stabilization = rf.check("stabilization", Value(false), "Stabilization mode on/off").asBool();

	bEveryThingisGood *= initPorts(rf);
	bEveryThingisGood *= initGaze(rf);
        
        head->setTrackingMode(tracking);
        
        head->setSaccadesMode(saccadic);
        
        head->setStabilizationMode(stabilization);
        
//         head->setTrackingMode(true);

	finishedGaze = true;
	
	return bEveryThingisGood; 
}

bool gaze::initGaze(yarp::os::ResourceFinder &rf){
	//For gaze
	
	Property optionsGaze;
	optionsGaze.put("device","gazecontrollerclient");
	optionsGaze.put("remote","/iKinGazeCtrl");
	optionsGaze.put("local","/client/gaze");

	robotHeadGaze = new PolyDriver(optionsGaze);
	head = NULL;
	if (robotHeadGaze->isValid()) {
		robotHeadGaze->view(head);
		cout<<"Head opened properly"<<endl;
	}
	
	return true;
}

bool gaze::initPorts(yarp::os::ResourceFinder &rf){
			    
	setName(moduleName.c_str());

	portOutName = "/";
	portOutName += getName() + "/out";

	if (!portOut.open(portOutName.c_str())) {           
            cout << getName() << ": Unable to open port " << portOutName << endl;  
            return false;
	}

	portInName = "/";
	portInName += getName() + "/input";

	if (!portIn.open(portInName.c_str())) {           
            cout << getName() << ": Unable to open port " << portInName << endl;  
            return false;
	}
	attach(portIn);

	return true;
}

void gaze::gazeTo(double x, double y, double z){
  
	yarp::sig::Vector explorationGaze(3), currentGazePosition(3);
        double euclideanDistance = 0;
        double trajectoryTime = 1;
    
	head->getFixationPoint(currentGazePosition);
        
	explorationGaze[0]=x;
	explorationGaze[1]=y;
	explorationGaze[2]=z; 
        
        euclideanDistance = (double)sqrt((double)pow(x-currentGazePosition[0],2) + (double)pow(y-currentGazePosition[1],2) + (double)pow(z-currentGazePosition[2],2));
        
  
        if(euclideanDistance > 0.4){
                trajectoryTime = 2;
        }

	if(head != NULL && finishedGaze && euclideanDistance > 0.05){
                head->setNeckTrajTime(trajectoryTime);
                
                head->clearNeckPitch();
                
		head->lookAtFixationPoint(explorationGaze);
                
		finishedGaze = false;
	}
//         sendGazeDirection(finishedGaze);
	
	head->getFixationPoint(currentGazePosition);
	
}

void gaze::sendGazeDirection(bool block){
        yarp::sig::Vector currentGazePosition(3);
    	head->getFixationPoint(currentGazePosition);
	
        Bottle gaze_direction;
        gaze_direction.addString("gaze");
        gaze_direction.addInt((int)block);
        gaze_direction.addDouble(currentGazePosition[0]);
        gaze_direction.addDouble(currentGazePosition[1]);
        gaze_direction.addDouble(currentGazePosition[2]);
        portOut.write(gaze_direction);
}


bool gaze::interruptModule() {
    return true;
}

bool gaze::close() {
	
	head->stopControl();

	delete head;

    return true;
}

bool gaze::respond(const Bottle& command, Bottle& reply) {
        string cmd = command.get(0).asString();
        if(cmd.compare("gazeTo") == 0){
                x = command.get(1).asDouble();
                y = command.get(2).asDouble();
                z = command.get(3).asDouble();
                finishedGaze = true;
                if(x < -0.2) //If the hand is not too close from the body
                    gazeTo(x, y, z);
        }
        reply.addString("Command received");
        return true;
}

/* Called periodically every getPeriod() seconds */
bool gaze::updateModule() {
    
	if(!finishedGaze && head->waitMotionDone(0.1)){
		finishedGaze = true;
		head->stopControl();
	}
// 	else if(finishedGaze){
//                 gazeTo(x, y, z);
//         }
	
//         sendGazeDirection(finishedGaze);
	
	return true;
}

double gaze::getPeriod() {
    /* module periodicity (seconds), called implicitly by myModule */    
    return 0.25;
}


