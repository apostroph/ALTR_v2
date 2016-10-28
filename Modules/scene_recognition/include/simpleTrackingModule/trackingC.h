#include <iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <deque>

#pragma once

using namespace std;
using namespace cv;

typedef cv::Point3_<double> Point3d;

class trackingC
{
public:
	trackingC(Point position, double color, double size);
		
	double isRecognized(Point position, double color, double size);
        
        bool isContained(Point position, double color, double size);
	
	bool update(Point position, double color, double size, Rect dimensions, std::vector<int> histogram, Point2d velocity_correction = Point2d(0., 0.));
	
	int step();
	
	bool isAlive();
	bool isFound();
	
	void drawTrajectory(Mat img);
	
	int getState();
	
	double getHue();
	
	Scalar getRGB();
	
	std::deque<Point> getTrajectory();
	
	Point getPosition(){return position_object;}
	
	Point2d getVelocity(){return velocity;}
	
	/**
         * Return string description of "size + color + shape"
         * 
         */
	string getDescription();
        
	
	/**
         * Return string of state
         * 
         */
        string getStateString();

	~trackingC(void);

private:	
	std::deque<Point> trajectory;
	
	Point position_object;
	
	double height;
        
        Point dimensions;
	
	Point2d velocity;
	
	Point2d dTraveled;

	double pTraveled;

	double approxSize;
	double meanColor;
        
        std::vector<int> histogram;
        
	bool recognized;
        double recognized_threshold;
	
	bool isVisible;
	bool isMoved;
	
	double recognitionThreshold;
	bool acquired;
	
	time_t timerStart, timerStop;
        
        template <typename T> T normal_pdf(T x, T m, T s);
};

