/* 
 * Copyright (C): None
 * Authors: Jimmy Baraglia
 * Public License for more details
*/

#include "simpleTrackingModule.h"

using namespace cv;

int main(int argc, char * argv[]) {
  
    VideoCapture capture(0);
   // capture = VideoCapture("../images_test/inputVideo.avi");
    
    cout<<"Main function started"<<endl;
    
    simpleTrackingModule visualModule = simpleTrackingModule();
    
    cv::Mat A = imread("../images_test/test_2.jpg", CV_LOAD_IMAGE_COLOR);
    
    while(1){
	  
	  capture>>A;
	  visualModule.newImage(A);
	  //Different getters
	  /*cout<<"So far, "<<visualModule.getNumberOfTrackedObject()<<" objects have been tracked"<<endl;
	  cout<<"Right now, "<<visualModule.getNumberOfActiveTrackedObject()<<" objects are tracked"<<endl;
	  
	  for(int c = 0; c < visualModule.getNumberOfTrackedObject(); c ++){
		if(visualModule.getBlobState(c) != -1){ // If this tracker is active
		      cout<<"The position of the blob "<<c<<" is: "<<visualModule.getPositionOfBlob(c)<<endl;
		      cout<<"The 3D position of the blob "<<c<<" is: "<<visualModule.get3DPositionOfBlob(c)<<endl;
		      cout<<"The RGB color of the blob "<<c<<" is: "<<visualModule.getRGBColorOfBlob(c)<<endl;
		      cout<<"The Hue of the blob "<<c<<" is: "<<visualModule.getHueColorOfBlob(c)<<endl;
		      cout<<"The number of point for the trajctory of blob "<<c<<" is: "<<visualModule.getTrajectory(c).size()<<endl;
		}
	  }*/
    }
    
    cout<<"Main function stopping"<<endl;

    return 0;
}

