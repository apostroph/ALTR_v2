/* 
 * Copyright (C): None
 * Authors: Jimmy Baraglia
 * Public License for more details
*/

#include "iCub/actionUtils.hpp"

namespace actionUtils
{

  float get_angle_of_line_between_two_points(yarp::sig::Vector p1, yarp::sig::Vector p2) {
      float xDiff = p2[0] - p1[0];
      float yDiff = p2[1] - p1[1];
      return atan2(yDiff, xDiff) * (180 / M_PI);
  }
  
  

  /**
  * Align current leading emotional state to motion parameters
  * /emotion_id ==> 0: neutral; 1: happy; 2: surprise; 3: fear; 4: disgust; 5: sadness; 6: anger;
  */
  void set_motion_parameters(Vector& motion_parameters, int emotion_id) {
      motion_parameters = Vector(3);
      switch(emotion_id){
	  case 1: motion_parameters[0] = 0.8; motion_parameters[1] = 0.7; motion_parameters[2] = 0.1; //happy
	      break;
	  case 2: motion_parameters[0] = 1; motion_parameters[1] = 1.0; motion_parameters[2] = 0.3; //surprised
	      break;
	  case 3: motion_parameters[0] = 0.6; motion_parameters[1] = 0.2; motion_parameters[2] = 0.9; //afraid
	      break;
	  case 4: motion_parameters[0] = 0.5; motion_parameters[1] = 0.6; motion_parameters[2] = 0.9; //disgusted
	      break;
	  case 5: motion_parameters[0] = 0.0; motion_parameters[1] = 0.3; motion_parameters[2] = 0.1; //sad
	      break;
	  case 6: motion_parameters[0] = 1.0; motion_parameters[1] = 0.9; motion_parameters[2] = 0.8; //angry
	      break;
	  default: motion_parameters[0] = 0.5; motion_parameters[1] = 0.8; motion_parameters[2] = 0.1; //neutral;
	      break;
      }
  }
  
  void send_gaze_target(yarp::os::Port* port, yarp::sig::Vector gaze_target){
        Bottle gaze_target_bottle;
        gaze_target_bottle.addString("gazeTo");
        gaze_target_bottle.addDouble(gaze_target[0]);
        gaze_target_bottle.addDouble(gaze_target[1]);
        gaze_target_bottle.addDouble(gaze_target[2]);
        gaze_target_bottle.addInt(3);
        port->write(gaze_target_bottle);
  }
  
  void get_motion_orientation(double Rx, double Ry, double Rz, yarp::sig::Vector* orientation){
        yarp::sig::Vector ox(4), oy(4), oz(4);
        
        ox[0]=1.0; ox[1]=0.0; ox[2]=0.0; ox[3]=Rx; //+M_PI/2.0;//Fore arm rotation
        
        oy[0]=0.0; oy[1]=1.0; oy[2]=0.0; oy[3]=Ry; //-M_PI/2.0; //
        
        oz[0]=0.0; oz[1]=0.0; oz[2]=1.0; oz[3]=Rz; //RZ2; // Wrist
        
        Matrix Rotx=yarp::math::axis2dcm(ox);
        Matrix Roty=yarp::math::axis2dcm(oy);        // from axis/angle to rotation matrix notation
        Matrix Rotz=yarp::math::axis2dcm(oz);
        Matrix R=Rotx*Roty*Rotz;// Compose the rotations keeping the order
        *orientation = yarp::math::dcm2axis(R);
    
  }
  
}