/* 
 * Copyright (C): None
 * Authors: Jimmy Baraglia
 * Public License for more details
*/
#include <string>

#include <yarp/sig/all.h>
#include <yarp/dev/all.h>
#include <yarp/os/all.h>
#include <yarp/math/Math.h>

#include <unordered_map>

#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <opencv/highgui.h>
#include <opencv/cv.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;

using namespace cv;

namespace perceptionUtils
{
    void color_threshold(Mat img, Mat* dst);
    void GetThresholdedImage(Mat imgHSV, Mat* dst, double color, int satMin = 120);
    
    void display_imgs(Mat img_left, Mat img_right, string name);
    
    
}