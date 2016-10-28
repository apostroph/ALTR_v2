#include "simpleTrackingModule/trackingC.h"

#define T_POSITION 1
#define T_COLOR 1

#define CHOSE_COLOR 0

#define RECOG_THRESH 7

#define MAX_QUEUE 10


trackingC::trackingC(Point in_position, double color, double size):
position_object(in_position), approxSize(size), meanColor(color), isVisible(true), 
isMoved(false), acquired(false), pTraveled(0), recognized(false)
{
	dTraveled.x = 0;
	dTraveled.y = 0;
	velocity.x = 0;
	velocity.y = 0;
        
        recognitionThreshold = 0;
	
	time(&timerStart);
	timerStop = timerStart;
}

double trackingC::isRecognized(Point position, double color, double size){        
        double euclidean_distance = sqrt((double)pow(position_object.x-position.x,2) + (double)pow(position_object.y-position.y,2));
        double color_euclidean = sqrt((double)pow(meanColor - color, 2));
        
        euclidean_distance = normal_pdf(euclidean_distance, 0., 30.)/normal_pdf(0., 0., 30.);
        color_euclidean = normal_pdf(color_euclidean, 0., 20.)/normal_pdf(0., 0., 20.);
        
        double recognition_strength = euclidean_distance * color_euclidean;
        
	if(recognized == true){
            recognition_strength = 0;
        }
        
	return recognition_strength;
}

bool trackingC::isContained(Point position, double color, double size){	
	bool contained = false;
		 
        double euclidean_distance = sqrt((double)pow(position_object.x-position.x,2) + (double)pow(position_object.y-position.y,2));
        double color_euclidean = sqrt((double)pow(meanColor - color, 2));
	
	if(euclidean_distance < 50){	  
                contained = true;
	}
	
	return contained;
}

bool trackingC::update(Point position, double color, double size, Rect dimensions, std::vector<int> histogram, Point2d velocity_correction){
    
        recognized = true;
                    
        //If the object has been detected
        isVisible = true;
        
        //The color of the object is updated
        this->meanColor = color;

        pTraveled = sqrt((double) ((dTraveled.x*dTraveled.x) + (dTraveled.y*dTraveled.y)));
        
        //The velocity is updated
        this->velocity.x = (position.x-this->position_object.x) - velocity_correction.x;
        this->velocity.y = (position.y-this->position_object.y) - velocity_correction.y;
        
        //The varaible for the motion detection is updated
        this->dTraveled.x = 0.90*this->dTraveled.x + this->velocity.x;
        this->dTraveled.y = 0.90*this->dTraveled.y + this->velocity.y;
        
        //The position is updated
        this->position_object.x = position.x;
        this->position_object.y = position.y;
        
        approxSize = size;	
        
        //The trajectory points if the object position changed
        trajectory.push_back(this->position_object);
        if(trajectory.size() > MAX_QUEUE){
                trajectory.pop_front();
        }
        
        //Update dimensions
//         this->dimensions = dimensions;
        this->histogram = histogram;
        
        
        //The object is detected in motion if the distance traveled is above 20
        if(pTraveled > 20){
                isMoved = true;
                
                dTraveled.x = dTraveled.y = 0;
        }
        else{
                isMoved = false;
        }
            
        return true;
}

int trackingC::step(){
	if(recognized){
		//If the object was recogized, the timer is initialized
		time(&timerStart);
		timerStop = timerStart;
                
                recognitionThreshold ++;
                if(recognitionThreshold > RECOG_THRESH){
                    recognitionThreshold = 7;
                    acquired = true;
                }
	}else{
                if(recognitionThreshold < RECOG_THRESH){
                    recognitionThreshold --;
                }
		//If the object was not recogized, the timer is updated
		time(&timerStop);
                
		if((double)(timerStop-timerStart) > 1){
			//If the object is no recognized for 1s, it becomes not visible
			isVisible = false;
                        
			trajectory.clear();
		}
		if((double)(timerStop-timerStart) > 3){
                        acquired = false;
                }
	}
	recognized = false;

	return 0;
}

void trackingC::drawTrajectory(Mat img){
	for(int count = 1; count < trajectory.size(); count++){
	    int thickness = 3;
	    line(img, trajectory[count-1], trajectory[count], Scalar(180, 0, 0), thickness);
	}
}

int trackingC::getState(){
        int state = 0;
	if(!acquired && !isVisible){
            state = -1;
        }else if(acquired){
            state = 1;
            if(!isVisible){
                state = 2;
            }else if(isMoved){
                state = 3;
            }
        }
        
        return state;
}

/**
* Return string of state
* 
*/
string trackingC::getStateString(){
        string state_s = "STA";
        
        if(getState() == 2){
            state_s = "OCC";
        }else if(getState() == 3){
            state_s = "MVG";
        }
        
        return state_s;
}

/**
* Return string description of "size + color + shape"
* 
*/
string trackingC::getDescription(){
        string description = "";
        
        if(approxSize > 40){
            description += "BIG_";
        }else{
            description += "SMALL_";
        }
        
        double number_colored_pixel = histogram[0] + histogram[1] + histogram[2];
        vector<string> colors = {"RED", "YELLOW", "BLUE"};
        for(auto count = 0; count < histogram.size(); count++){
            if(((double)histogram[count]/number_colored_pixel) > 0.1){
                description += colors[count]+"_";
            }
        }
        description += "OBJ";
        
        return description;
}

bool trackingC::isAlive(){
     return acquired;
}


bool trackingC::isFound(){
     return recognized;
}
 
 
double trackingC::getHue(){
      return meanColor;
}

Scalar trackingC::getRGB(){
      double R, G, B;
      if(meanColor <= 60){
	      R = 255;
	      G = 4.25*meanColor;
	      B = 0;
      }else if(meanColor <= 120){
	      R = 255-(4.25*(meanColor-60));
	      G = 255;
	      B = 0;
      }else if(meanColor <= 180){
	      R = 0;
	      G = 255;
	      B = (4.25*(meanColor-120));
      }
      Scalar RGB(R,G,B);
      return RGB;
}

template <typename T> T trackingC::normal_pdf(T x, T m, T s)
{
    static const T inv_sqrt_2pi = 0.3989422804014327;
    T a = (x - m) / s;

    return inv_sqrt_2pi / s * std::exp(-T(0.5) * a * a);
}

std::deque<Point> trackingC::getTrajectory(){
      return trajectory;
}


trackingC::~trackingC(void)
{
}
