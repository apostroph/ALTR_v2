/*
 * Copyright: (C) 2012 iCub Facility, Istituto Italiano di Tecnologia
 * Author: Lorenzo Natale
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */

#include <yarp/os/Time.h>
#include <string>
#include <iostream>
#include <yarp/os/RFModule.h>

#include "iCub/actionRepertoire.h"

using namespace std;
using namespace yarp::os;

int goal1, goal2, act1, act2, R11, R12, R21, R22, RG1, RG2; 


bool actionRepertoire::configure(yarp::os::ResourceFinder &rf) {
    
	//Set the module name
	moduleName = rf.check("name", Value("actionRepertoire"), "module name (string)").asString();
	setName(moduleName.c_str());
	
	//Set the robot name	    
	robotName = rf.check("robot", Value("icub")).asString();
        
        //Enable or Desable arms
        rightArmEnabled = rf.check("rightArm", Value(true)).asBool();
        leftArmEnabled  = rf.check("leftArm",  Value(true)).asBool();
	
	//Open the different openPorts
	openPorts(rf);
	
	//Open the left and right arm cartesianController
	openControllers(rf);

        //Set the limit for the motions
        init_limits(rf);

        motion_parameters = yarp::sig::Vector(3);
        emotion_id = 1;
        actionUtils::set_motion_parameters(motion_parameters, emotion_id);
        
        
        //Loops ensure that the module does not start while the home position is not acquired
        if(rightArmEnabled){
            do{
                cartesianCtrlRight->getPose(home_pose_right, home_rot_right);
            }while(home_pose_right[0] == 0);
        }else{
            cartesianCtrlRight = NULL;
        }
        
        if(leftArmEnabled){
            do{
                cartesianCtrlLeft->getPose(home_pose_left, home_rot_left);
            }while(home_pose_left[0] == 0);
        }else{
            cartesianCtrlLeft = NULL;
        }      

        return true;
}

bool actionRepertoire::openPorts(yarp::os::ResourceFinder &rf){
	//init the network
	yarp = new Network();
	Network::init();
	
	//Open Acknolegement port
	string portAckName = "/";
	portAckName += getName() + "/out";

	if (!portAck.open(portAckName.c_str())) {           
	    cout << getName() << ": Unable to open port " << portAckName << endl;  
	    return false;
	}
	cout << "Port ack openned, please connect it" << endl;
	
	//Open input port
	string portInputName = "/";
	portInputName += getName() + "/input";

	if (!portInput.open(portInputName.c_str())) {           
	    cout << getName() << ": Unable to open port " << portInputName << endl;  
	    return false;
	}
	
	attach(portInput);
    
	return true;
}

bool actionRepertoire::openControllers(yarp::os::ResourceFinder &rf){	
  
	string remoteNameRight= "/"+robotName+"/cartesianController/right_arm";
	string remoteNameLeft= "/"+robotName+"/cartesianController/left_arm";

	string localNameRight="/client/right_arm";
	string localNameLeft="/client/left_arm";
	
        if(rightArmEnabled){
            cout  << "Opening right arm controller" <<endl;
            if(openCartCon(localNameRight, remoteNameRight, &clientCartCtrlRight, &cartesianCtrlRight, &dofRight) == false){
                rightArmEnabled = false;
            }
            cartesianCtrlRight->getRestPos(rest_right);
        }else{
            cout<<"Right arm is desabled"<<endl;
        }
	
        if(leftArmEnabled){
            cout  << "Opening left arm controller" <<endl;
            if(openCartCon(localNameLeft, remoteNameLeft, &clientCartCtrlLeft, &cartesianCtrlLeft, &dofLeft) == false){
                leftArmEnabled = false;
            }
            cartesianCtrlLeft->getRestPos(rest_left);
        }else{
            cout<<"Left arm is desabled"<<endl;
        }
	
	return true;
}

bool actionRepertoire::openCartCon(string localName, string remoteName, PolyDriver **clientCartCtrl, ICartesianControl **cartesianCtrl, yarp::sig::Vector* dof){
	Property option("(device cartesiancontrollerclient)");
	option.put("remote",remoteName.c_str());
	option.put("local",localName.c_str());	

	*clientCartCtrl = new PolyDriver(option);

	*cartesianCtrl = NULL;

	if ((*clientCartCtrl)->isValid()) {
		(*clientCartCtrl)->view(*cartesianCtrl);
		cout<<localName<<" cartesianControl Valid"<<endl;
	}else{
		cout<<localName<<" cartesianControl non Valid"<<endl;
		return false;
	}
	   
	(*cartesianCtrl)->setPosePriority("position");

	(*cartesianCtrl)->setTrajTime(1);
	(*cartesianCtrl)->getDOF(*dof);
	
        enableBody((*cartesianCtrl), dof);        

	return true;
}

double actionRepertoire::set_traj_time(ICartesianControl* icart, double dist){
    trajectory_time = 10;
    double mult = 1.;
    if(emotion_id == 3 || emotion_id ==6){ //surprised or angry
        mult = 0.25;
    }else if(emotion_id == 0 || emotion_id == 1){ //neutral or happy
        mult = 1;
    }else{ //disgusted, afraid or sad
        mult = 3;
    }
    trajectory_time *= mult*dist;

    trajectory_time <= 1 ? trajectory_time = 1 : trajectory_time;

    icart->setTrajTime(trajectory_time);
    
    return trajectory_time;
}

void actionRepertoire::init_limits(yarp::os::ResourceFinder &rf){
	limits = yarp::sig::Vector(6);
	
	limits[0] = rf.find("x_min").asDouble();
	limits[1] = rf.find("x_max").asDouble();
	
	limits[2] = rf.find("y_min").asDouble();
	limits[3] = rf.find("y_max").asDouble();
	
	limits[4] = rf.find("z_min").asDouble();
	limits[5] = rf.find("z_max").asDouble();
}

bool actionRepertoire::respond(const Bottle& command, Bottle& reply) {
        string target = command.get(0).asString();
	string cmd = command.get(1).asString();
        yarp::sig::Vector object(3), dist(3);
                
        yarp::sig::Vector home_gaze = yarp::sig::Vector(3);
        home_gaze[0] = -0.4; home_gaze[1] = 0; home_gaze[2] = 0.1;
        
	if(target.compare("action") != 0) {
                return true;
        }else if (cmd.compare("quit") == 0){
		return false;
	}
	else if(cmd.compare("home") == 0) {
            
                //Executes the going home motion
                if(rightArmEnabled){
                    go_home(cartesianCtrlRight, home_pose_right, home_rot_right);
                }
                if(leftArmEnabled){
                    go_home(cartesianCtrlLeft, home_pose_left, home_rot_left);
                }
                
                //Send the position of the object to the gaze module
                actionUtils::send_gaze_target(&portAck, home_gaze);
                
                //Reply for TCP (UDP?) acknoledgement
		reply.addString("Going home");
	}
	else if(cmd.compare("push") == 0 ){
                
                //Gets the position of the target object
                object[0] = command.get(2).asDouble();  object[1] = command.get(3).asDouble();  object[2] = command.get(4).asDouble();
                
                //Gets the position of the target to push the object to
                dist[0] = command.get(5).asDouble();  dist[1] = command.get(6).asDouble();  dist[2] = command.get(7).asDouble();
                
                //Executes the pushing motion
                if(push_to(object, dist)){
                    
                    //Acknoledge to the prediction module
                    acknowledge("Push done");
                
                    //Send the position of the object to the gaze module
                    actionUtils::send_gaze_target(&portAck, home_gaze);
                }else{
                    
                    //Acknoledge to the prediction module
                    acknowledge("Push failed");
                }
                //Reply for TCP (UDP?) acknoledgement
                reply.addString("Push command received");
	}
	else if(cmd.compare("touch") == 0 ){
                
                //Gets the position of the target to push the object to
                dist[0] = command.get(2).asDouble();  dist[1] = command.get(3).asDouble();  dist[2] = command.get(4).asDouble();
                
                //Executes the touching motion
                if(touch(dist)){
                    
                    //Acknoledge to the prediction module
                    acknowledge("Touch done");
                
                    //Send the position of the object to the gaze module
                    actionUtils::send_gaze_target(&portAck, home_gaze);
                }else{
                    
                    //Acknoledge to the prediction module
                    acknowledge("Touch failed");
                }
                //Reply for TCP (UDP?) acknoledgement
                reply.addString("Push command received");
                
	}
	else if(cmd.compare("wave") == 0){
                bool side = 0;
                
                //Gets the value relative to the motion side
                string cmd_side = command.get(2).asString();
                
                if(cmd_side.compare("right") == 0){
                    side = 1;
                }
            
                //Executes the waving motion
                if(wave(side)){
                    
                    //Acknoledge to the prediction module
                    acknowledge("Wave done");
                
                    //Send the position of the object to the gaze module
                    actionUtils::send_gaze_target(&portAck, home_gaze);
                }else{
                    
                    //Acknoledge to the prediction module
                    acknowledge("Wave failed");
                }
        }else if(cmd.compare("move") == 0){
                
                //Gets the position of the target to push the object to
                dist[0] = command.get(2).asDouble();  dist[1] = command.get(3).asDouble();  dist[2] = command.get(4).asDouble();
                
                
                for(auto motion = 0; motion < 5; motion ++){
                    goal1 = 1; goal2 = 0; act1 = 1; act2 = 0;
                    R11 = 1; R12 = 0; R21 = 0; R22 = 0; RG1 = 1; RG2 = 0; 
                    move_arm_to(cartesianCtrlRight, -0.45, 0.3, 0, M_PI, 0, M_PI);
                    
                    goal1 = 0; goal2 = 0; act1 = 0; act2 = 1;
                    R11 = 0; R12 = 1; R21 = 0; R22 = 0; RG1 = 0; RG2 = 1; 
                    go_home(cartesianCtrlRight, home_pose_right, home_rot_right);
                    
                    goal1 = 0; goal2 = 1; act1 = 1; act2 = 0;
                    R11 = 0; R12 = 0; R21 = 1; R22 = 0; RG1 = 1; RG2 = 0; 
                    move_arm_to(cartesianCtrlRight, -0.45, -0.05, 0, M_PI, 0, M_PI);
                    
                    goal1 = 0; goal2 = 0; act1 = 0; act2 = 1;
                    R11 = 0; R12 = 0; R21 = 0; R22 = 1; RG1 = 0; RG2 = 1; 
                    go_home(cartesianCtrlRight, home_pose_right, home_rot_right);
                    
                }
                
                cout<<"OVER!!!!!!!!!!!!"<<endl;
                
                go_home(cartesianCtrlRight, home_pose_right, home_rot_right);
        }
	else if(cmd.compare("emotion") == 0) {
            
                //Gets the value relative to the emotion
		emotion_id = command.get(2).asInt();
                
                //Sets the emotion ID to the utils module
                actionUtils::set_motion_parameters(motion_parameters, emotion_id);
                
                //Reply for TCP (UDP?) acknoledgement
            	reply.addString("Set emotion");
                
	}else if(cmd.compare("test_position") == 0){
                bool doable = true;
                dist[0] = command.get(2).asDouble();  dist[1] = command.get(3).asDouble();  dist[2] = command.get(4).asDouble();
                ICartesianControl* iCart = (dist[1] > 0) ? cartesianCtrlRight : cartesianCtrlLeft; 
                
                doable = inRange(dist) && (iCart != NULL);
                
                reply.addInt(doable);
        }
	else {
		reply = command;
	}
	return true;
}

/*
 * True if the planned motion in inside the reachable, false else
 **/
bool actionRepertoire::inRange(yarp::sig::Vector p1){
        //If the target are within the limits
	if(	(p1[0] >= limits[0] 	&& p1[0] <= limits[1]) &&
		(p1[1] >= limits[2] 	&& p1[1] <= limits[3]) &&
		(p1[2] >= limits[4]+0.02 && p1[2] <= limits[5])){
		return true;
	}
	cout<<"Out of reach"<<endl;
	return false;
}

bool actionRepertoire::push_to(yarp::sig::Vector object, yarp::sig::Vector dest, double strength){
    bool motionPossible = false;
    yarp::sig::Vector pre_push_target(3);
    double push_angle = 0;
    
    //selects the correct hand in function of the object position
    ICartesianControl* iCart = (object[1] > 0) ? cartesianCtrlRight : cartesianCtrlLeft; 
    bool right_Nleft = (object[1] > 0) ? true : false; 
    
    motionPossible = (iCart != NULL && inRange(object) && inRange(dest));
    
    //If the object and the destination are in range
    if(motionPossible){
        iCart->setPosePriority("orientation");
        
        //Calculates angle between the object position and the desitination
        double angle = (90+actionUtils::get_angle_of_line_between_two_points(object, dest));
        
        //Sets some limits to the angle
        angle = (std::abs(angle) > 45) ? (45*(std::abs(angle)/angle)) : angle;        
        
        //Calculate the pushing angle (in radian)
        push_angle = (M_PI*angle)/360;

        //Hand position positioned behind the object
        pre_push_target[0] = object[0]+0.03*cos(angle); pre_push_target[1] = object[1]+0.03*sin(angle); pre_push_target[2] = object[2];
    
        //Send the position of the object to the gaze module
        actionUtils::send_gaze_target(&portAck, (pre_push_target+dest)/2);

        //Pre-push initiated
        move_arm_to(iCart, pre_push_target[0], pre_push_target[1], pre_push_target[2],  M_PI/2, push_angle, M_PI);

        //Push initiated
        move_arm_to(iCart, dest[0], dest[1], dest[2],  M_PI/2, push_angle, M_PI);

        //Pre-push initiated
        move_arm_to(iCart, pre_push_target[0], pre_push_target[1], pre_push_target[2],  M_PI/2, push_angle, M_PI);
        
        iCart->setPosePriority("position");
        
        //go Home
        cout<<right_Nleft<<endl;
        if(right_Nleft)
            go_home(iCart, home_pose_right, home_rot_right);
        else
            go_home(iCart, home_pose_left, home_rot_left);
    }
    else{
        cerr<<"Push object or destination position out of range"<<endl;
    }
    return motionPossible;
}

bool actionRepertoire::wave(bool right_Nleft, float intensity, int nb_wave){
    bool motionPossible = false;
    yarp::sig::Vector wave_in(3), wave_out(3);
        
    //selects the correct hand in function of the object position
    ICartesianControl* iCart = (right_Nleft) ? cartesianCtrlRight : cartesianCtrlLeft; 
    
    motionPossible = (iCart != NULL);
    
    if(motionPossible){
        
        //sets position values for wave in
        wave_in[0] = -0.3;      wave_in[1] = (0.2-.09*intensity)*(2*(int)right_Nleft-1);     wave_in[2] = 0.5; 
        
        //sets position values for wave out
        wave_out[0] = -0.3;     wave_out[1] = (0.2+.09*intensity)*(2*(int)right_Nleft-1);    wave_out[2] = 0.5; 
        
        //sets the hand orientation as more important than the position
        iCart->setPosePriority("orientation");
        
        for(int c = 0; c < nb_wave; c++){
            //Wave in
            move_arm_to(iCart, wave_in[0], wave_in[1], wave_in[2], M_PI*(1 - (int)right_Nleft), (-M_PI/2)*(2*(int)right_Nleft-1), -(0.6*intensity));
            
            //Wave out
            move_arm_to(iCart, wave_out[0], wave_out[1], wave_out[2], M_PI*(1 - (int)right_Nleft), (-M_PI/2)*(2*(int)right_Nleft-1), (0.6*intensity));
        }
        
        //sets the hand position as more important than the orientation
        iCart->setPosePriority("position");
        
        //go Home
        if(right_Nleft)
            go_home(iCart, home_pose_right, home_rot_right);
        else
            go_home(iCart, home_pose_left, home_rot_left);
    }
    
    return motionPossible;
    
}

bool actionRepertoire::touch(yarp::sig::Vector dest){
    bool motionPossible = false;
    yarp::sig::Vector pre_touch(3);   
    
    //selects the correct hand in function of the destination position
    ICartesianControl* iCart = (dest[1] > 0) ? cartesianCtrlRight : cartesianCtrlLeft; 
    bool right_Nleft = (dest[1] > 0) ? true : false; 
    
    motionPossible = (iCart != NULL && inRange(dest));
    
    if(motionPossible){
        //sets position values for wave in
        pre_touch[0] = dest[0]; pre_touch[1] = dest[1]; pre_touch[2] = dest[2]+0.07; 
        
        //sets the hand orientation as more important than the position
        iCart->setPosePriority("orientation");
        
        //Send the position of the object to the gaze module
        actionUtils::send_gaze_target(&portAck, dest);
        
        //Pre touch
        move_arm_to(iCart, pre_touch[0], pre_touch[1], pre_touch[2], M_PI*((int)right_Nleft), 0, M_PI);
        
        //1.5s pause
        Time::delay(1.5);
        
        //Touch
        move_arm_to(iCart, dest[0], dest[1], dest[2], M_PI*((int)right_Nleft), 0, M_PI);
        
        //1s pause
        Time::delay(1);
        
        //Pre touch
        move_arm_to(iCart, pre_touch[0], pre_touch[1], pre_touch[2], M_PI*((int)right_Nleft), 0, M_PI);
        
        //sets the hand position as more important than the orientation
        iCart->setPosePriority("position");
        
        //go Home
        if(right_Nleft)
            go_home(iCart, home_pose_right, home_rot_right);
        else
            go_home(iCart, home_pose_left, home_rot_left);
    }
    return motionPossible;
}

    void actionRepertoire::go_home(ICartesianControl *ctrl, Vector home_pos, Vector home_rot){    
            //Create & store the contect
            int context;
            ctrl->storeContext(&context);
            
            //Set new DoF for contect
            Vector dof;
            ctrl->getDOF(dof); dof = 1.0;
            ctrl->setDOF(dof,dof);
            
            //Set limits 
            ctrl->setLimits(0,0.0,0.0);
            ctrl->setLimits(1,0.0,0.0);
            ctrl->setLimits(2,0.0,0.0);

            //Motion time = 3 seconds
            ctrl->setTrajTime(3);
            
            //Go to pose
            ctrl->goToPose(home_pos, home_rot);
            ctrl->waitMotionDone(0.1,4);
            
            //Restore the previous context
            ctrl->restoreContext(context);
            ctrl->deleteContext(context);
    }

void actionRepertoire::move_arm_to(ICartesianControl *icart, double x, double y, double z, double Rx, double Ry, double Rz){
    yarp::sig::Vector target(3);
    yarp::sig::Vector orientation(3);
    
    target[0] = x, target[1] = y; target[2] = z;
    
    actionUtils::get_motion_orientation(Rx, Ry, Rz, &orientation);
    
    if(inRange(target)){
        //The motion is executed twice to be sure it is done
        //If the motion if done correctly, it will take only few millisecond to run through the second action
        for(auto i= 0; i < 2; i++)
            execute_motion(icart, target, orientation);
//         cout<<"Motion exectued at" << target[0] << " : " << target[1] << " : " << target[2] << endl;
    }
}

void actionRepertoire::execute_motion(ICartesianControl *icart, yarp::sig::Vector target, yarp::sig::Vector orientation)
{
    Vector hp, od;
    double dist = 0;

    //Get current position and orientation  of the hand
    icart->getPose(hp,od); 

    //Calculate the euclidian distance between the hand and the target
    dist = sqrt(pow(hp[0]-target[0], 2) + pow(hp[1]-target[1], 2) + pow(hp[2]-target[2], 2)); 
    
    //Set the correct trajectory time based on the current velocity parameter
    double espected_duration = set_traj_time(icart, dist);
        
    //Execute the motion
    icart->goToPose(target, orientation);   // send request and forget
    
    //Wait until motion is done or exceed duration_time
    waitUntilTimeOut(2*espected_duration, icart);
	
}

bool actionRepertoire::waitUntilTimeOut(double espected_duration, ICartesianControl* icart){
    //Wait until motion is done or exceed duration_time
    bool done = false;
    float timeout = 0;
    while (!done && timeout < espected_duration) {
            icart->checkMotionDone(&done);
            Time::delay(0.04);   // or any suitable delay
            timeout += 0.04;
    }
}

bool actionRepertoire::enableBody(ICartesianControl* cartesianCtrl, yarp::sig::Vector* dof){
	Vector newDof(3);
	
	newDof[0]=1; // torso pitch: 1 => enable
	newDof[1]=0; // torso roll: 0 => disable
	newDof[2]=1; // torso yaw: 1 => enable
	
        cartesianCtrl->setLimits(0,0.0,30.0);
        cartesianCtrl->setLimits(1,0.0,0.0);
        cartesianCtrl->setLimits(2,-20.0,20.0);
	
	
	return cartesianCtrl->setDOF(newDof,*dof);
}

bool actionRepertoire::disableBody(ICartesianControl* cartesianCtrl, yarp::sig::Vector* dof){
	Vector newDof(3);
	
	newDof[0]=0; // torso pitch: 1 => disable
	newDof[1]=0; // torso roll: 2 => disable
	newDof[2]=0; // torso yaw: 1 => disable
	
	return cartesianCtrl->setDOF(newDof,*dof);
}

bool actionRepertoire::interruptModule() {
    cout << "Interrupting your module, for port cleanup" << endl;
    return true;
}

void actionRepertoire::acknowledge(string message){
    Bottle right_hand_pos_bottle;
    right_hand_pos_bottle.addString("action");
    right_hand_pos_bottle.addString(message);
    portAck.write(right_hand_pos_bottle);
}

bool actionRepertoire::updateModule() {
//     Vector hp, od;
//     double dist = 0;
// 
//     //Get current position and orientation  of the hand
//     cartesianCtrlRight->getPose(hp,od); 
// 
//     
//     cout<<goal1<<"\t"<<goal2<<"\t"<<act1<<"\t"<<act2<<"\t";
//     cout<<hp[0]<<"\t"<<hp[1]<<"\t"<<hp[2];
//     cout<<R11<<"\t"<<R12<<"\t"<<R21<<"\t"<<R22<<"\t"<<RG1<<"\t"<<RG2<<endl;
    
    return true;
}

bool actionRepertoire::close() {
    cout << "Calling close function\n";
    return true;
}

double actionRepertoire::getPeriod() {
    return 0.05;
}
