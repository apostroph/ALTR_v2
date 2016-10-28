/* 
 * Copyright: 2013 Robotcub Consortium
 * Author: Konstantinos Theofilis
 * Email: k.theofilis@herts.ac.uk
 * Copyright Policy: Released under the terms of the GNU GPL v2.0
 */

#include <string>
#include <yarp/os/RFModule.h>
#include <yarp/os/Network.h>

#include <eigen3/Eigen/Dense>

#include <opencv/highgui.h>
#include <opencv/cv.h>

#include "iCub/predictionUtils.hpp"

using namespace std;
using namespace cv;
using namespace yarp::os;

class Prediction: public RFModule
{
    public: 
    
    double getPeriod();
    
    bool updateModule();

    bool respond(const Bottle& command, Bottle& reply);

    bool configure(yarp::os::ResourceFinder &rf);
    
    bool openPorts(yarp::os::ResourceFinder &rf);

    bool interruptModule();

    bool close();
    
private:   
    /*Private functions*/
    
    /**
     * initialize_probabilities
     * Initialize the conditional probabilities used for prediction
     * 
     */
    void initialize_probabilities();
    
    void loadData(string src);
    
    void newEvidences(string &predicted_state, string &predicted_action, double &prediction_marg_probability, string state = "", string action = "");
    
    void getStates(ifstream &input);
    
    void getActions(ifstream &input);
    
    void getEdges(ifstream &input);
    
    bool isNewState(string new_state);
    
    void displayPrediction(vector<string> list_of_states, vector<double> list_of_predictions);
    
    void sendActionCmd(string action_cmd, Point3d position_obj, Point3d target = Point3d(0, 0, 0));
    
    void trainRobot();
    
    bool checkActionOnObject(Point3d position);
    
    
    /* class parameters */
    Network *yarp;
    
    yarp::os::Port portAck; //Output port
    yarp::os::Port portInput; //Input port
    
    /* module parameters */
    string moduleName;
    string robotName;

    //Prediction variables
    bool learning;
    bool waiting_for_action_result;
    
    double curiosity;
    double certainty;
    
    int current_looked_at_state;
    int current_perfomed_action;
    vector<string> observed_related_states;
    
    //Position of the hand for helping other condtion
    cv::Point3d hand_position;
    
    //List of states from the recognition module
    vector<string> current_states;
    vector<Point3d> current_states_position;
    
    //List fo states from and for the xml file
    vector<string> states; 
    unordered_map<int, string> index_to_state;
    unordered_map<string, int> state_to_index;  
    
    //List fo actions from and for the xml file
    vector<string> actions;
    unordered_map<int, string> index_to_action;
    unordered_map<string, int> action_to_index;  
    
    //List fo edges from and for the xml file
    vector< vector<string> > edges;
    
    //Network variables
    //Conditional probabilities calculated with maximum likelihood parameter function
    Eigen::MatrixXf state0_to_action0;
    Eigen::MatrixXf state0_and_action0_to_state1;
    
    //Marginal probabilities
    Eigen::VectorXf state0;
    Eigen::VectorXf action0;
    Eigen::VectorXf state1;
    
    //Training limits
    Eigen::VectorXf training_limit;
    

};
