/* 
 * Copyright (C): None
 * Authors: Jimmy Baraglia
 * Public License for more details
*/

#include "simpleTrackingModule/simpleTrackingModule.h"

//the size fo the images is divided by this value
#define SUB 1

//Size of object tracking trajectory window 
#define sizeTrajectory 10

//Variables for the skin removale
#define RED_MIN 95
#define GREEN_MIN 40
#define BLUE_MIN 20
#define MAX_MIN_SPAN 25
#define MIN_VALUE 15

#define TABLE_HEIGHT 0.1

#define MAX_TRACK_NB 50

#define CUT_HEIGHT 0.45


simpleTrackingModule::simpleTrackingModule():
sL(100), sH(255), vL(20), vH(255), hueL(0), hueH(255), medianBlurV(12), open(1), close(4)
{
	
	initBlobDetector("../initialization_files/blobDetector.ini");
		
	//namedWindow("RGB2", CV_WINDOW_AUTOSIZE+WINDOW_OPENGL);
	
	timeStep = 0;
}

/** newImage
 * Call this method to update the tracking
 * In:
 * cv::Mat RGB: RGB image 
 * cv::Mat depth: gray scale image containing the depth information
 * 
 * Out: NONE
 * 
 * */
void simpleTrackingModule::newImage(cv::Mat RGB, cv::Mat* mask){
	Mat smallImage;
	Mat RGBsmall;
	Mat smallImageGray;
        Point2d correction(0., 0.);
	
	//The image matrices are initialized
	smallImageGray = Scalar(0, 0, 0);
	smallImage = Scalar(0, 0, 0);
	
	//If the image isn't empty
	if(!RGB.empty()){
		RGB.copyTo(inputRGB);
		
		//The image matrice is resize to improve the computational time
		resize(inputRGB, smallImage, Size((inputRGB.cols)/SUB, (inputRGB.rows)/SUB));		
		smallImage.copyTo(RGBsmall);
                
		if(timeStep != 0){                        
                        correction = getAverageOpticalFlows(RGBsmall, pastImage);
                }
                               
		//The image goes through a complex thresholding 
		thresholdF(smallImage, smallImageGray);	
		
		//The blobs are detected
		blobDetector(smallImageGray);
                
                imshow("Extract", smallImageGray);
               
		for(auto count = 0; count < keypoints.size(); count ++){
			bool assigned = false;
                    
                        Mat roi(smallImage, blob_dimensions[count]);
                        
                        double color = getMeanColor(roi);

			if(color != -1){
                                vector<double> recognition_coef_list;
                                
				for(auto cT = 0; cT < list_of_objects.size(); cT++){
                                        double recognition_coef = list_of_objects[cT][0].isRecognized(SUB*keypoints[count].pt, color, keypoints[count].size);
                                        recognition_coef_list.push_back(recognition_coef);
				}
				double max_value = 0.3;
                                int index_max = -1;
                                
                                for(auto i = 0; i < recognition_coef_list.size(); i++){
                                    if(recognition_coef_list[i] > max_value && recognition_coef_list[i] > 0.6){
                                        max_value = recognition_coef_list[i];
                                        index_max = i;
                                    }
                                }
				if(index_max != -1){
                                        assigned = true;
                                        std::vector<int> histogram = getColorHistogram(roi);
                                        list_of_objects[index_max][0].update(SUB*blob_positions[count], color, keypoints[count].size, blob_dimensions[count], histogram, SUB*correction);
                                }
				
				if(!assigned){
                                    
                                        bool containedBlob = false;
					for(auto cT = 0; cT < list_of_objects.size(); cT++){
						if(list_of_objects[cT][0].isContained(blob_positions[count], color, keypoints[count].size)){
							containedBlob = true;
						}
					}
					if(!containedBlob){
						trackingC newTracker = trackingC(blob_positions[count], color, keypoints[count].size);
                                                
						std::deque<trackingC> newTrajectory;
						newTrajectory.push_front(newTracker);
						list_of_objects.push_back(newTrajectory);
                                        }
				}
			}
		}
		
		smallImageGray.copyTo((*mask));
		
		for(auto cT = 0; cT < list_of_objects.size(); cT++){
			list_of_objects[cT][0].step();
                        
			if(list_of_objects[cT][0].getState() < 0){
			    list_of_objects.erase(list_of_objects.begin()+cT);
			    cT--;
			}
		}
		
		RGBsmall.copyTo(pastImage);
		timeStep ++;
		waitKey(1);
		
	}else{
		cerr<<"Error RGB image not found"<<endl;
	}
}

Point2d simpleTrackingModule::getAverageOpticalFlows(Mat current_img, Mat past_img){
        Point2d averageFlow(0, 0);
        int width =current_img.cols;
        int height = current_img.rows;
        
        vector<Point2f> featuresPrevious;
        vector<Point2f> featuresCurrent;
        vector<uchar> featuresFound;
        Mat err;
        
        for(auto c_flow_w = 0; c_flow_w < 10; c_flow_w ++){
            for(auto c_flow_h = 0; c_flow_h < 10; c_flow_h ++){
                //all the tracking positions
                Point2f new_point(((width/10) * c_flow_w)+width/20, ((height/10) * c_flow_h)+height/20);
                featuresPrevious.push_back(new_point);
                    
            }
        }
        
        calcOpticalFlowPyrLK(past_img,current_img,featuresPrevious,featuresCurrent,featuresFound,err); //problem line 
        
         
        double nb_flow = featuresCurrent.size();
        for(auto point = 0; point < nb_flow; point++){
//             line(past_img, featuresCurrent[point], featuresPrevious[point], Scalar(255, 0, 255) ,2);
            
            averageFlow.x +=  featuresCurrent[point].x - featuresPrevious[point].x;
            averageFlow.y +=  featuresCurrent[point].y - featuresPrevious[point].y;
            
        }
        averageFlow.x /= nb_flow;
        averageFlow.y /= nb_flow;
//         imshow("Optical", past_img);
        
        return averageFlow;
}


/** saturationThreshold
 * Remove high saturation values
 * in:
 * Mat src: source image
 * int min: HSV min saturation
 * int max: HSV max saturation
 * 
 * out:
 * Mar dst: destination image
 * */
void simpleTrackingModule::thresholdF(Mat src, Mat &dst){
	
	Mat noSkin = cv::Mat(Size((src.cols), (src.rows)), CV_8UC3);  
	Mat colorBlobs = cv::Mat(Size((src.cols), (src.rows)), CV_8UC3);  
	
	skinDetection(src, noSkin);//We remove the skin color
        
        src.copyTo(src, noSkin);
	
	//The matrice pixels are convert from BGR to HSV
	cvtColor(noSkin, noSkin, CV_RGB2HSV);
	
	inRange(noSkin, Scalar(hueL,sL,vL), Scalar(hueH,sH,vH), dst); 
		
 	medianBlur ( dst, dst, medianBlurV*3+1 );
	
	erode(dst, dst, cv :: Mat (), cv :: Point ( - 1 , - 1 ), 5 );
 	dilate(dst, dst, cv :: Mat (), cv :: Point ( - 1 , - 1 ), 3 );
	
	
	//Add a black border to the image
	rectangle(dst, Point(0, 0), Point(dst.cols, dst.rows*CUT_HEIGHT), Scalar(0, 0, 0), -1);
	rectangle(dst, Point(0, dst.rows*CUT_HEIGHT), Point(dst.cols, dst.rows), Scalar(0, 0, 0), 20);
	
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;
	
	//Find the contour of all the shapes
	findContours(dst, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
	
	dst = Scalar(0);
	
	//If the object size is too small, the contour is erased
	for(vector<vector<Point> >::iterator it = contours.begin(); it != contours.end();){
		if(it->size() < (30/pow(SUB,2))){
		      it = contours.erase(it);
		}else{
		      ++it;
		}
	}
	
	//All the shapes are filled in white
	for(auto i = 0; i < contours.size(); i++){		
		drawContours(dst, contours, i, Scalar(255), 3);
	}
	
	dst = Scalar(0);
	
	//All the shapes are filled in white
	for(auto i = 0; i < contours.size(); i++){		
		drawContours(dst, contours, i, Scalar(255), -1);
		drawContours(dst, contours, i, Scalar(0), 3);
	}
	
	//The dimension of each contour is saved;
	blob_dimensions.clear();
        blob_positions.clear();
        int left, right, top, down;
        for(auto i = 0; i < contours.size(); i++){
            left = INT_MAX; right = INT_MIN;
            top = INT_MAX; down = INT_MIN;
            Point new_position;
            for(auto contour_p:contours[i]){
                left  = contour_p.x < left  ? contour_p.x : left;
                right = contour_p.x > right ? contour_p.x : right;
                top   = contour_p.y < top   ? contour_p.y : top;
                down  = contour_p.y > down  ? contour_p.y : down;
                new_position.x += contour_p.x;
                new_position.y += contour_p.y;
            }
            Rect new_dim = Rect(Point(right, down), Point(left, top));
            blob_dimensions.push_back(new_dim);
            
            new_position.x /= contours[i].size();
            new_position.y /= contours[i].size();
            blob_positions.push_back(new_position);
        }
	
// 	imshow("Gray image", dst);
	
}

/** Skin color threshold
 * Remove skin color
 * Was removed due to the fact that it removes yellow and red also
 
 * */
void simpleTrackingModule::skinDetection(Mat src, Mat &dst){//const unsigned char red, const unsigned char green, const unsigned char blue, const unsigned char max, const unsigned char min)
        
        dst = Scalar(0, 0, 0);
	for(auto c = 0; c < src.cols; c++){
		for(auto r = 0; r < src.rows; r++){
			Vec3b pixelColor = src.at<Vec3b>(Point(c,r));
			int maxV = std::max(pixelColor[2], std::max(pixelColor[1], pixelColor[0]));
			int minV = std::min(pixelColor[2], std::min(pixelColor[1], pixelColor[0]));
			
			if((pixelColor[2] > RED_MIN && pixelColor[1] > GREEN_MIN && pixelColor[0] > BLUE_MIN && (unsigned int)(maxV - minV) > MAX_MIN_SPAN &&
				(unsigned int)std::abs(pixelColor[2] - pixelColor[1]) > MIN_VALUE && 
				pixelColor[2] > pixelColor[1] && pixelColor[2] > pixelColor[0]))
			{		
			      dst.at<Vec3b>(Point(c,r)) = pixelColor;
			}
		}
	}
	
 	dilate(dst, dst, cv :: Mat (), cv :: Point ( - 1 , - 1 ), 5 );
// 	erode(dst, dst, cv :: Mat (), cv :: Point ( - 1 , - 1 ), 5 );
        
	medianBlur ( dst, dst, medianBlurV*3+1 );
	
	erode(dst, dst, cv :: Mat (), cv :: Point ( - 1 , - 1 ), 5 );
 	dilate(dst, dst, cv :: Mat (), cv :: Point ( - 1 , - 1 ), 3 );
        
        vector<vector<Point> > contours;
        vector<Vec4i> hierarchy;
        
        rectangle(dst, Point(0, 0), Point(dst.cols, dst.rows), Scalar(0, 0, 0), 5);
        
        Mat canny_output;
        
        Canny( dst, canny_output, 100, 200, 3 );
//         findContours( canny_output, contours, hierarchy,CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE )
        
        findContours(canny_output, contours, hierarchy, CV_RETR_EXTERNAL,  CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
        
        int largest_contour = 0;
        int index_max = -1;
        
        for(auto index = 0; index < contours.size(); index++){
                if(contours[index].size() > largest_contour){
                        bool hand_detected = false;
                        for(auto pixel_pos = 0; pixel_pos < contours[index].size(); pixel_pos++){
                                Vec3f intensity = dst.at<Vec3b>(contours[index][pixel_pos]);
                                if((float)intensity.val[0] >= 10 && contours[index][pixel_pos].y < src.rows*CUT_HEIGHT){
                                        hand_detected = true;
                                        break;
                                }
                        }
                        if(hand_detected){
                                largest_contour = contours[index].size();
                                index_max = index;
                        }
                }
        }
        
        dst = Scalar(0,0, 0);
        
        
        Point height_min;
        double left_most = INT_MAX;
        double right_most = INT_MIN;
        int y = INT_MIN;
        
        if(index_max != -1){            
                for(auto x = 0; x <  contours[index_max].size(); x++){
                        if(contours[index_max][x].y > y){
                                y = contours[index_max][x].y;
                                height_min = contours[index_max][x];
                        }
                        if(contours[index_max][x].x > right_most)
                                right_most = contours[index_max][x].x;
                        else if(contours[index_max][x].x < left_most)
                                left_most = contours[index_max][x].x;
                }
                hand_position= height_min;
        
                src.copyTo(dst);
                drawContours(dst, contours, index_max, Scalar(0, 0, 0), -1);
        }else{
                hand_position.x = 0;
                hand_position.y = 0;
        }
//         rectangle(dst, Point(left_most, 0), Point(right_most, height_min.y), Scalar(0, 0, 0), CV_FILLED);
}

/** getBlobColor
 * extract the mean Hue color of an area
 * In:
 * Mat image: input color
 * */
double simpleTrackingModule::getBlobColor(Mat imgBlob, Mat imgRGB, double size, Point position){
	Mat newImg = cv::Mat(Size(imgBlob.cols, imgBlob.rows), CV_8UC3);
	resize(imgRGB, newImg, newImg.size());
      
	
	double meanColor = 0;	
	if((position.x - size) > 0 && (position.y - size) > 0 && (position.x+size) < newImg.cols && (position.y+size) < newImg.rows){
                Mat roi(newImg, Rect(position.x - size, position.y - size, 2*size, 2*size));

                meanColor = getMeanColor(roi);
	}
	
	return meanColor;
}

/** getMeanColor
 * extract the mean color of an area
 * In:
 * Mat image: input image
 * */
double simpleTrackingModule::getMeanColor(Mat ROI){
      double meanColor = 0;
      double nbValue = 0;
      
      for(auto c = 0; c < ROI.cols; c++){
		for(auto r = 0; r < ROI.rows; r++){
			Vec3b pixelColor = ROI.at<Vec3b>(Point(c,r));
			meanColor+= (double)pixelColor[0];
			nbValue ++;
		}
	}
	meanColor /= nbValue;
        
//         imshow(to_string(meanColor), ROI);
        
	return meanColor;
}
     
     
/**
* Return the witdh and height
* in: indexB in the list of tracked object
* */
Point simpleTrackingModule::getSizefBlob(Mat ROI){
        vector<vector<Point> > contours;
        vector<Vec4i> hierarchy;
        Point dimension;

        //change to black and white
	inRange(ROI, Scalar(hueL,sL,vL), Scalar(hueH,sH,vH), ROI); 
        
        //Find the contour of all the ROI
        findContours(ROI, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
        
        return dimension;
}


/** getMeanColor
 * extract the mean color of an area
 * In:
 * Mat image: input image
 * */
std::vector<int> simpleTrackingModule::getColorHistogram(Mat ROI){
      std::vector<int> histogram;
      
      for(auto color = 0; color<3; color++)
            histogram.push_back(0);
      
      //histogram[0] ==> red = 10-20
      //histogram[1] ==> yellow = 40-50
      //histogram[2] ==> blue = 125-135
      
//       cout<<ROI.cols<<" : "<<ROI.rows<<endl;
      
      cvtColor(ROI, ROI, CV_BGR2RGB);
      
      int max_hue = 0;
      
      for(auto c = 0; c < ROI.cols; c++){
		for(auto r = 0; r < ROI.rows; r++){
			Vec3b pixelColor = ROI.at<Vec3b>(Point(c,r));
//                         cv::Scalar color = getRGB((double)pixelColor[0]);
                        
//                         cout<<(int)pixelColor[0]<<" :: "<<(int)pixelColor[1]<<" :: "<<(int)pixelColor[2]<<endl;
                        
                        double color_average = ((int)pixelColor[0] + (int)pixelColor[1] + (int)pixelColor[2])/3;
                        
                        if(color_average > 50){
                                if((int)pixelColor[0] > color_average && (int)pixelColor[1] < color_average && (int)pixelColor[2] < color_average){ //High low low 
                                    histogram[0]++;
                                }else if((int)pixelColor[0] > color_average && (int)pixelColor[1] > color_average && (int)pixelColor[2] < color_average){ //High high low
                                    histogram[1]++;
                                }else if((int)pixelColor[0] < color_average && (int)pixelColor[1] > color_average/2. && (int)pixelColor[2] > color_average){ //low high high
                                    histogram[2]++;
                                }
                        }
		}
	}
// 	cout<<max_hue<<endl;
//         cout<<histogram[0]<<" ::  "<<histogram[1 ]<<" :: "<<histogram[2]<<endl;
	
	return histogram;
}
     

/** blobDetector
 * Extract all the blob contained in the argument image
 * In:
 * Mat image: input color
 * */
void simpleTrackingModule::blobDetector(Mat image){
	// ... any other params you don't want default value
	Point point;
	point.x = 0;
	point.y = 0;

	// detect!
	blob_detector.detect(image, keypoints);
}

/**
 * Method to initialize the blob detector
 * For more information see simpleBlobDector datasheet
 * 
 * The parameters of the blob detector 
 * are editable from ./initialization_files/blobDetector.ini
 * */
bool simpleTrackingModule::initBlobDetector(const char* src){
	bool everyThingIsOk = true;
	string line;
	double parameters[8];
	int index = 0;
	
	cout<<"Blob detector initialization starting"<<endl;
	
	ifstream myfile(src);
	
	if (myfile.is_open())
	{
		while ( getline (myfile,line) )
		{
			if(line.find("#") == std::string::npos && line.find("=") != std::string::npos){
				parameters[index] = string_to_double(line.substr(line.find("=")+1));
				index++;	
			}
			
		}
		myfile.close();
	}else
		cerr<<"Initialization file not found at "<<src<<endl;
		
	if(index == 8){
		params.minDistBetweenBlobs = (double)parameters[0];
		params.filterByInertia = (bool)parameters[1];
		params.filterByConvexity = (bool)parameters[2];
		params.filterByColor = (bool)parameters[3];
		params.filterByCircularity = (bool)parameters[4];
		params.filterByArea = (bool)parameters[5];
		params.minArea = (double)parameters[6]/(pow(SUB,2));
		params.maxArea = (double)parameters[7]/(pow(SUB,2));
		
		blob_detector = SimpleBlobDetector( params );
		cout<<"Blob detector parmaters correctly set"<<endl;
	}else{
		cerr<<"Blob detector initilization: number of argument invalid"<<endl;
	}
	
	if(!everyThingIsOk){
		cerr<<"Error during the blob detector initialization"<<endl;
	}
	cout<<"Blob detector initialization finished"<<endl;
	return everyThingIsOk;
}

//Get methods
     
/**
* Return the number of tracked objects
* The objects no longer observervable are also counted
* */
int simpleTrackingModule::getNumberOfTrackedObject(){
	return list_of_objects.size();  
}

/**
* Return the number of active tracked objects
* The objects no longer observervable are not counted
* */
int simpleTrackingModule::getNumberOfActiveTrackedObject(){
	int value = 0;
	for(auto c = 0; c < list_of_objects.size(); c++){
		if(list_of_objects[c][0].getState() >= 1){
			value ++;
		}
	}  
	return value;
}

/**
* Return the state of the tracked object at indexB
* -1 => Gone
* 0 => Occluded
* 1 => stationary
* 2 => Moving
* 
* in: indexB in the list of tracked object
* */
int simpleTrackingModule::getBlobState(int indexB){
	 if(list_of_objects.size() > indexB && indexB >= 0){
		return list_of_objects[indexB][0].getState();
	 }
	 return -1;
}

/**
* Return the 3D position of the tracked object at indexB
* in: indexB in the list of tracked object
* */
Point3d simpleTrackingModule::get3DPositionOfBlob(int indexB){
    Point3d position_world;
    Point position_object;
    
    //Modify when using with real iCub
    //Improvement could include automatic estimation of the transformation matrix
    double w11 = 0.0011962; double w12 = 0.0015217;
    double w21 = -0.35167; double w22 = -0.66609;
    
    if(list_of_objects.size() > indexB && indexB >= 0){
        position_object = list_of_objects[indexB][0].getPosition();
        
        position_world.x = position_object.y*w12 + w22;
        position_world.y = position_object.x*w11 + w21;
        position_world.z = TABLE_HEIGHT;                
        
    }
    return position_world;
}
     
/**
* Return the 3D position of the thand
* */
Point3d simpleTrackingModule::get3DPositionOfHand(){
    Point3d position_hand_3d;
    position_hand_3d = Point3d(0, 0, 0);
    
    //Modify when using with real iCub
    //Improvement could include automatic estimation of the transformation matrix
    double w11 = 0.0011962; double w12 = 0.0015217;
    double w21 = -0.35167; double w22 = -0.66609;
          
    if(hand_position.x != 0 && hand_position.y != 0 ){
        position_hand_3d.x = hand_position.y*w12 + w22;
        position_hand_3d.y = hand_position.x*w11 + w21;
        position_hand_3d.z = TABLE_HEIGHT; 
    }
        
    return position_hand_3d;
}

/**
* Return the 2D position of the tracked object at indexB
* in: indexB in the list of tracked object
* */
Point simpleTrackingModule::getPositionOfBlob(int indexB){
	 if(list_of_objects.size() > indexB && indexB >= 0){
		return list_of_objects[indexB][0].getPosition();
	 }
	 return Point(0,0);
}

/**
* Return the RGB color of the tracked object at indexB
* in: indexB in the list of tracked object
* */
Scalar simpleTrackingModule::getRGBColorOfBlob(int indexB){
	 if(list_of_objects.size() > indexB && indexB >= 0){
		return list_of_objects[indexB][0].getRGB();
	 }
	 return Scalar(0, 0, 0);
}

/**
* Return the Hue of the tracked object at indexB
* in: indexB in the list of tracked object
* */
double simpleTrackingModule::getHueColorOfBlob(int indexB){
	 if(list_of_objects.size() > indexB && indexB >= 0){
		return list_of_objects[indexB][0].getHue();
	 }
	 return -1;
}

/**
* Return the trajectory of the tracked object at indexB
* in: indexB in the list of tracked object
* */
deque<Point> simpleTrackingModule::getTrajectory(int indexB){
	 if(list_of_objects.size() > indexB && indexB >= 0){
		return list_of_objects[indexB][0].getTrajectory();
	 }
	 deque<Point> temp;
	 return temp;
}

/**
* Return the 2D position of the hand
* Point(0, 0) = no hand
* */
Point simpleTrackingModule::getPositionOfHand(){
        return hand_position*SUB;
}

/**
* Return the string description of the objecx indexB
* */
string simpleTrackingModule::getObjectDescription(int indexB){
        string description = "";
        //description containes color(s), size and shapes
        
        if(list_of_objects.size() > indexB && indexB >= 0){
                description = list_of_objects[indexB][0].getDescription();
        }
        
        return description;
}


/**
* Return the string description of the objecx indexB
* */
string simpleTrackingModule::getObjectState(int indexB){
        string state_s = "";
        //state_s containes state
        
        if(list_of_objects.size() > indexB && indexB >= 0){
                state_s = list_of_objects[indexB][0].getStateString();
        }
        
        return state_s;
}


simpleTrackingModule::~simpleTrackingModule()
{
}

//Other methods

/**
 * Convert string to double
 * In: 
 * const std::string s: string to convert to double
 * Out: double contained the argument, "nan" if string not a number
 * */
double simpleTrackingModule::string_to_double(const std::string& s){
   std::istringstream i(s);
   double x;
   if (!(i >> x))
     return std::numeric_limits<double>::quiet_NaN();
   return x;
 } 
 
 cv::Scalar simpleTrackingModule::getRGB(double hue){
      double R, G, B;
      if(hue <= 60){
	      B = 255;
	      G = 4.25*hue;
	      R = 0;
      }else if(hue <= 120){
	      B = 255-(4.25*(hue-60));
	      G = 255;
	      R = 0;
      }else if(hue <= 180){
	      B = 0;
	      G = 255;
	      R = (4.25*(hue-120));
      }
      cv::Scalar RGB(R,G,B);
      return RGB;
 }
