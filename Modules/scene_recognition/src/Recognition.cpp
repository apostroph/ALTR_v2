/* 
 * Copyright (C): None
 * Authors: Jimmy Baraglia
 * Public License for more details
*/

#define QUEUE_SIZE 5

#define PERIOD 0.05

#include "iCub/Recognition.h"


bool Recognition::configure(yarp::os::ResourceFinder &rf)
{    
    
	//init the network
	Network::init();
        
	moduleName = rf.check("name", Value("perception"), "module name (string)").asString();
	setName(moduleName.c_str());
	
	initPorts(rf);
	
	//Init the blob tracker
	tracker = simpleTrackingModule();
        
        string image_source = rf.check("source", Value("cam")).asString();
        image = false;
        webcam = false; 
        icub_cam = false;
        
        if(image_source.compare("image") == 0){
            string test_image = rf.check("image", Value("e1")).asString();
            test_img_src = "../data/test_images/"+test_image+".jpg";
            image = true;
                    
        }else if(image_source.compare("video") == 0){
            string video_file = rf.check("video", Value("")).asString();
            if(video_file == ""){
                capture = VideoCapture(0); // open the video camera no. 0
            }else{
                video_file = "../data/test_video/"+video_file+".mp4";
                capture = VideoCapture(video_file); // open the video camera no. 0
            }
            if (!capture.isOpened())  // if not success, exit program
            {
                cout << "Cannot open the video cam" << endl;
            }else
                webcam = true;
        }else{
            icub_cam = true;
        }
        
        //Initiciliazation of program specific varibles
        nb_act = 0;
        gazing = false;
        
	return true;
        
        
}

bool Recognition::initPorts(yarp::os::ResourceFinder &rf){

	// Open cam left
	camLeftName = "/";
	camLeftName += getName() + "/in/left";

	if (!camLeft.open(camLeftName.c_str())) {           
	    cout << getName() << ": Unable to open port " << camLeftName << endl;  
	    return false;
	}

	// Open IN
	portInName = "/";
	portInName += getName() + "/input";

	if (!portIn.open(portInName.c_str())) {           
		cout << getName() << ": Unable to open port " << portInName << endl;  
		return false;
	}

	// Open OUT
	portOutName = "/";
	portOutName += getName() + "/out";

	if (!portOut.open(portOutName.c_str())) {           
		cout << getName() << ": Unable to open port " << portOutName << endl;  
		return false;
	}

	attach(portIn);
	
	return true;
}

void Recognition::sendInformation(){
	Bottle visionOutBottle;	
        cv::Point3d object_pos;
        int trackedObjects = tracker.getNumberOfTrackedObject();
        
        visionOutBottle.addString("stateList");
        visionOutBottle.addInt(tracker.getNumberOfActiveTrackedObject());
        
        for(auto object_count = 0; object_count < trackedObjects; object_count++){
                if(tracker.getBlobState(object_count) > 0){
                    //Get real states
                    object_pos = tracker.get3DPositionOfBlob(object_count);
                    
                    visionOutBottle.addString(tracker.getObjectDescription(object_count));
                    visionOutBottle.addString(tracker.getObjectState(object_count));
                    visionOutBottle.addDouble(object_pos.x); visionOutBottle.addDouble(object_pos.y); visionOutBottle.addDouble(object_pos.z);
                }
        }
        
        cv::Point3d hand = tracker.get3DPositionOfHand();
        visionOutBottle.addDouble(hand.x); visionOutBottle.addDouble(hand.y); visionOutBottle.addDouble(hand.z);
        
        portOut.write(visionOutBottle);
}

/* Called periodically every getPeriod() seconds */
bool Recognition::updateModule() {
        Mat img_saving = cv::Mat(Size(640, 480), CV_8UC3);  
        if(image){
            newImg = imread(test_img_src, CV_LOAD_IMAGE_COLOR);
        }else if(webcam){
            capture.read(newImg);             
        }else{
            imgLeft = camLeft.read();
            newImg = Mat((IplImage*)imgLeft->getIplImage(), true);
            cvtColor(newImg, newImg, CV_BGR2RGB);
        }
    
    
        if(!gazing){
            Mat mask;
            tracker.newImage(newImg, &mask);
            
            drawObjects();
            
            addObjectInfo();
            addFPS();
            imshow("Result", newImg);
            
            
            resize(newImg, img_saving, img_saving.size());
            writer.write(img_saving);
            
            nb_act++;
            if((double)(nb_act*PERIOD) >= 0.25){
                sendInformation();
                nb_act = 0;
            }
        }
	
	waitKey(1);
	
    return true;
}

double Recognition::getPeriod() {
    /* module periodicity (seconds), called implicitly by myModule */    
    return PERIOD;
}

bool Recognition::interruptModule() {
    return true;
}

bool Recognition::close() {
//     writer.release();
    return true;
}

bool Recognition::respond(const Bottle& command, Bottle& reply) {
    string cmd = command.get(0).asString();
    if (cmd.compare("gaze") == 0) {
        cout<<command.get(1).asInt()<<endl;
            if(command.get(1).asInt() == 0){
                gazing = true;
            }else{
                gazing = false;
            }
    }
    reply.addString("Command received");
    return true;
}

void Recognition::drawObjects(){
	for(int count1 = 0; count1 < tracker.getNumberOfTrackedObject(); count1++){
                int state_v = tracker.getBlobState(count1);
		if(state_v >= 1){
                    std::stringstream ss;
                    
                    
                    if(state_v == 1)
                        circle(newImg, tracker.getPositionOfBlob(count1), 10, Scalar(255, 0, 0, 127), 3);
                    else if(state_v == 2)
                        circle(newImg, tracker.getPositionOfBlob(count1), 10, Scalar(0, 0, 255, 127), 3);
                    else
                        circle(newImg, tracker.getPositionOfBlob(count1), 10, Scalar(0, 255, 0, 127), 3);
                        
                    tracker.getTrackedObject(count1)->drawTrajectory(newImg);
                      
                    ss << "Object "<<(int)count1<<" : "<<tracker.getObjectDescription(count1);
                    string s_FPS = ss.str();
                    putText(newImg, s_FPS, Point(tracker.getPositionOfBlob(count1).x, tracker.getPositionOfBlob(count1).y), CV_FONT_HERSHEY_COMPLEX, 0.4, Scalar(255,0,0));
		}
	}
	
	if(tracker.getPositionOfHand().x != 0){
                circle(newImg, tracker.getPositionOfHand(), 10, Scalar(0, 255, 255), 3);
        }
    
}

void Recognition::addFPS(){
	end = yarp::os::Time::now();
	std::stringstream ss;
	ss << "FPS: "<<(int)(1/(end-begin));
	string s_FPS = ss.str();
	putText(newImg, s_FPS, Point(10, 10), CV_FONT_HERSHEY_COMPLEX, 0.4, Scalar(0,0,0));
		
	
	begin = yarp::os::Time::now();
	end = begin;
}

void Recognition::addObjectInfo(){	
	std::stringstream ss;
	ss << "NB_Obj: "<<(int)tracker.getNumberOfActiveTrackedObject();
	string msg = ss.str();
	putText(newImg, msg, Point(200, 10), CV_FONT_HERSHEY_COMPLEX, 0.4, Scalar(0,0,0));
}

