/*
 * Copyright: (C) 2012 iCub Facility, Istituto Italiano di Tecnologia
 * Author: Lorenzo Natale
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */

#include <yarp/os/Time.h>
#include <string>
#include <iostream>
#include <yarp/os/RFModule.h>
#include "iCub/Prediction.h"

using namespace std;
using namespace yarp::os;

bool Prediction::respond(const Bottle& command, Bottle& reply) {
    if (command.get(0).asString() == "quit") {
        return false;
    }
    else if (command.get(0).asString() == "stateList") {
        //Clear the list of observed states for update
        current_states.clear();
        current_states_position.clear();
        
        for(auto state_count = 0; state_count < command.get(1).asInt(); state_count++){
            //states are in command.get(2+state_count).asString()
            string object_description = command.get(2+(5*state_count)).asString();
            string object_state = command.get(2+1+(5*state_count)).asString();
            
            if(state_to_index[object_description+"_"+object_state] == 0){
                states.push_back(object_description+"_"+object_state);
                
                state_to_index[object_description+"_"+object_state] = states.size()-1; index_to_state[states.size()-1] = object_description+"_"+object_state;
                
                initialize_probabilities();
            }
            
            if(!waiting_for_action_result){
                Point3d newPos;
                bool already_included = false;
                string new_object_descr = object_description+"_"+object_state;
                
                for(auto current_obj:current_states){
                    if(new_object_descr.compare(current_obj) == 0){
                        already_included = true;
                        break;
                    }
                }
                
                if(!already_included){
                    newPos = Point3d(command.get(2+2+(5*state_count)).asDouble(), command.get(2+3+(5*state_count)).asDouble(), command.get(2+4+(5*state_count)).asDouble());
                    current_states.push_back(new_object_descr);
                    current_states_position.push_back(newPos);
                }
            }else{
                //If the state description is close enough to be the same basic object
                if(index_to_state[current_looked_at_state].compare(object_description+"_"+object_state) != 0 && abs(object_description.compare(index_to_state[current_looked_at_state])) <= 5){
                    observed_related_states.push_back(object_description+"_"+object_state);
//                     cout<<observed_related_states<<endl;
                }
            }
        }
        
        hand_position.x = command.get(2+0+5*command.get(1).asInt()).asDouble(); 
        hand_position.y = command.get(2+1+5*command.get(1).asInt()).asDouble(); 
        hand_position.z = command.get(2+2+5*command.get(1).asInt()).asDouble(); 
        
    }
    else if (command.get(0).asString() == "action") {
        current_states.clear();
        
        //Here call the method to analyse the observed_related_states
        string edge_state0 = index_to_state[current_looked_at_state];
        string edge_action = index_to_action[current_perfomed_action];
        string edge_state1 = edge_state0;
        
//         cout<< edge_state0 << " : " << edge_action << " : " << edge_state1 <<endl;
        vector<int> most_observed_state;
        for(auto state:states){
            most_observed_state.push_back(0);
        }
        if(observed_related_states.size() > 0){
            //The most observed one
            for(auto obs_state:observed_related_states){
                most_observed_state[state_to_index[obs_state]] ++;
            }
            
            auto max_element = std::distance(observed_related_states.begin(), std::max_element(observed_related_states.begin(), observed_related_states.end()));
            edge_state1 = observed_related_states[(int)max_element];
        }
        
        cout<<" ------------------------------------------------------------------"<<endl;
        cout<<edge_state0<<" : "<<edge_action<<" : "<<edge_state1<<endl;
        if(edge_state0.compare("NONE") != 0 && edge_action.compare("NONE") != 0 && edge_state1.compare("NONE") != 0){
        
            vector<string> new_edge;
            new_edge.push_back(edge_state0); new_edge.push_back(edge_action); new_edge.push_back(edge_state1);
            edges.push_back(new_edge);
                    
            initialize_probabilities();
        }
        cout<<" ------------------------------------------------------------------"<<endl;
        
        
        waiting_for_action_result = false;
    }else if(command.get(0).asString() == "save"){
        predictionUtils::saveData("experiment", states, actions, edges);
    }else if(command.get(0).asString() == "save"){
        learning = command.get(1).asBool();
    }
    else {
        reply = command;
    }
    return true;
}

bool Prediction::configure(yarp::os::ResourceFinder &rf) {
    //Set the module name
    moduleName = rf.check("name", Value("prediction"), "module name (string)").asString();
    setName(moduleName.c_str());
    
    //Should the module trains its task model
    learning = rf.check("learning", Value(1)).asInt();
    waiting_for_action_result = false;
    
    //Set the robot name	    
    robotName = rf.check("robot", Value("icub")).asString();
    
    //Open the different openPorts
    openPorts(rf);
    
    string file = rf.check("file", Value("init")).asString();
    loadData("../data/"+file+".xml");
    cout<<endl;
    
    //Variable for learning
    curiosity = rf.check("curiosity", Value(0.67)).asDouble();
    
    certainty = rf.check("certainty", Value(0.5)).asDouble();
    certainty = certainty > 1 ? 1 : certainty;
    certainty = certainty < 0 ? 0 : certainty;
    
    initialize_probabilities();
    
    return true;
}

void Prediction::initialize_probabilities(){    
    
    //Initialization of matrices//
    //Initialize the conditional probaility matrix P(A(t)|S(t))
    state0_to_action0 = Eigen::MatrixXf::Zero(actions.size(), states.size());
    
    //Initialize the conditional probaility matrix P(S(t+1)|S(t),A(t))
    state0_and_action0_to_state1 = Eigen::MatrixXf::Zero(states.size(), actions.size()*states.size());
    
    //Initialize the marginal probaility vector P(S(t))
    state0 = Eigen::VectorXf::Zero(states.size());
    
    //Initialize the marginal probaility vector P(A(t))
    action0 = Eigen::VectorXf::Zero(actions.size());
    
    //Initialize the marginal probaility vector P(S(t+1))
    state1 = Eigen::VectorXf::Zero(states.size());
    
    //Initialize the training limit vector
    training_limit = Eigen::VectorXf::Zero(actions.size());
    
    for(auto action_count = 0; action_count < actions.size(); action_count++)
            training_limit(action_count) = 0.3;
    
    if(edges.size() != 0){
        //Estimate conditional probabilities//
        //Sum of all event occurence
        for(auto edge_i:edges){
            int state0_i = 0, action0_i = 0, state1_i = 0;
            int action0_i_state0_i = 0;
            
            state0_i = state_to_index[edge_i[0]];
            action0_i = action_to_index[edge_i[1]];
            state1_i = state_to_index[edge_i[2]];
            
            state0_to_action0(action0_i, state0_i) += 1./(3.*(actions.size()-1.));
            
            action0_i_state0_i = action0_i + actions.size()*state0_i;
            state0_and_action0_to_state1(state1_i, action0_i_state0_i) += 1;
        }
        
        //Mean of  event occurence
        for(auto action_count = 0; action_count < actions.size(); action_count++){
            //Get the sum of the col for A(t) = action_count of P(A(t)|S(t))
            double state0_to_action0_sum = state0_to_action0.row(action_count).sum();
            
            if(state0_to_action0_sum > 1){
                //Normalize so that sum(P(A(t) = action_count|S(t))) = 1;
                state0_to_action0.row(action_count) /= state0_to_action0_sum;
                training_limit(action_count) = 0.3/state0_to_action0_sum;
            }
        }
            
        for(auto state_count = 0; state_count < states.size()*actions.size(); state_count++){
            //Get the sum of the col for A(t),S(t) = action_count + nb_action*state_count of P(S(t+1)|S(t),A(t))
            double state0_and_action0_to_state1_sum = state0_and_action0_to_state1.col(state_count).sum();
            
            if(state0_and_action0_to_state1_sum > 0){
                //Normalize so that sum(P(S(t+1) = state_count|S(t),A(t))) = 1;
                state0_and_action0_to_state1.col(state_count) /= state0_and_action0_to_state1_sum;
            }
        }
    }
    
}

void Prediction::newEvidences(string &predicted_state, string &predicted_action, double &prediction_marg_probability, string state, string action){
    //Marginal probabilities    
//     cout<<state_to_index[state]<<endl;
//     cout<<action_to_index[action]<<endl<<endl;
    
    
    //Initialize the marginal probaility vector P(S(t))
    state0 = Eigen::VectorXf::Zero(states.size());
    
    //Initialize the marginal probaility vector P(A(t))
    action0 = Eigen::VectorXf::Zero(actions.size());
    
    //Initialize the marginal probaility vector P(S(t+1))
    state1 = Eigen::VectorXf::Zero(states.size());
    
    //The conditional probaility matrix P(A(t)|S(t)) must be normalized
    Eigen::MatrixXf state0_to_action0_norm = Eigen::MatrixXf(actions.size(), states.size());
    state0_to_action0_norm = state0_to_action0;
    //Mean of  event occurence
    for(auto action_count = 0; action_count < actions.size(); action_count++){
        //Get the sum of the col for A(t) = action_count of P(A(t)|S(t))
        double state0_to_action0_sum = state0_to_action0_norm.row(action_count).sum();
        
        if(state0_to_action0_sum > 0){
            //Normalize so that sum(P(A(t) = action_count|S(t))) = 1;
            state0_to_action0_norm.row(action_count) /= state0_to_action0_sum;
        }
    }
    
    predicted_state = "";
    predicted_action = "";
    prediction_marg_probability = 0;
    
    if(state_to_index[state] > 0){
            state0(state_to_index[state]) = 1;
            
            //P(A(t)) = P(A(t)|S(t)) * P(S(t))
            action0 = state0_to_action0_norm * state0;
            if(action0.sum() > 0)
                action0 /= action0.sum(); 
            
            //P(S(t+1)) = P(S(t+1)|S(t),A(t)) * P(A(t), S(t)) = P(S(t+1)|S(t),A(t)) * P(A(t)|S(t)) * P(S(t))
            for(auto state_count_1 = 1; state_count_1 < states.size(); state_count_1++){
                if(state_count_1 != state_to_index[state]){
                    for(auto state_count_2 = 1; state_count_2 < states.size(); state_count_2++){
                        double p1, p2;
                        double p3 = state0(state_count_2);
                        if(action_to_index[action] > 0){
                            p1 = state0_and_action0_to_state1(state_count_1, action_to_index[action] + actions.size()*state_count_2);
                            p2 = state0_to_action0_norm(action_to_index[action], state_count_2);
                            
                            state1(state_count_1) += p1 * p2 * p3;
                        }else{
                            for(auto action_count = 1; action_count < actions.size(); action_count++){
                                p1 = state0_and_action0_to_state1(state_count_1, action_count + actions.size()*state_count_2);
                                p2 = state0_to_action0_norm(action_count, state_count_2);
                                
                                state1(state_count_1) += p1 * p2 * p3;
                            }
                        }
                    }
                }else{
                    state1(state_count_1) += 0;
                }
            }
            
            //If the sum of all state proba is higher than 1, we normalize
            if(state1.sum() > 1){
                state1 /= state1.sum(); 
            }
            
            int most_likely_future_state;
            state1.maxCoeff(&most_likely_future_state);
            if(most_likely_future_state == state_to_index[state]){
                state1(most_likely_future_state) = 0;
                state1.maxCoeff(&most_likely_future_state);
            }
        
            //Need to check if the maxCoeff always send a value
            
            Eigen::VectorXf possible_actions = Eigen::VectorXf(actions.size());
            for(auto action_count = 0; action_count < actions.size(); action_count++){
                possible_actions(action_count) = state0_and_action0_to_state1(most_likely_future_state, action_count + actions.size()*state_to_index[state]);
            }
            if(possible_actions.sum() > 0){
                possible_actions /= possible_actions.sum(); 
            }
            
            int most_likely_action;
            possible_actions.maxCoeff(&most_likely_action);
            
            
            predicted_state = index_to_state[most_likely_future_state];
            predicted_action = index_to_action[most_likely_action];
            prediction_marg_probability = state1[most_likely_future_state];
            
    }
//     cout<<endl<<" ---------------------------------------- "<<endl<<endl;
}

bool Prediction::openPorts(yarp::os::ResourceFinder &rf){
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

void Prediction::loadData(string src){
    ifstream input(src);
    string line;
    
    if (input.is_open())
    {
            cout<<"Loading "<<src<<endl;
            while ( getline (input, line) )
            {
                    if(line.find("<StateSpace>") != std::string::npos){
                            getStates(input);
                    }else if(line.find("<ActionList>") != std::string::npos){
                            getActions(input);
                    }else if(line.find("<EdgeList>") != std::string::npos){
                            getEdges(input);
                    }
            }
            input.close();
    }else{
            cerr<<"Initialization file not found at "<<src<<endl;
    }
    cout<<"Loading over"<<endl;
}

void Prediction::getStates(ifstream &input){
        string line = "";
        getline (input, line);
        
        cout<<"Loading state space"<<endl;
        
        while(line.find("</StateSpace>") == std::string::npos){
            states.push_back(line);
            index_to_state[states.size()-1] = line;
            state_to_index[line] = states.size()-1;
            
            getline (input, line);
        }
}

void Prediction::getActions(ifstream &input){            
        string line = "";
        getline (input, line);
        
        cout<<"Loading action space"<<endl;
        
        while(line.find("</ActionList>") == std::string::npos){
            actions.push_back(line);
            index_to_action[actions.size()-1] = line;
            action_to_index[line] = actions.size()-1;
            
            getline (input, line);
        }
}

void Prediction::getEdges(ifstream &input){
        string line = "";
        getline (input, line);
        cout<<"Loading action/state sequences"<<endl;
        
        while(line.find("</EdgeList>") == std::string::npos){
            vector<string> one_edge;
            int index = 0;
            
            while(line.find(',') != std::string::npos){
                index = line.find(',');
                one_edge.push_back(line.substr(0, index));
                
                line = line.substr(index+1);
            }
            one_edge.push_back(line);
            
            edges.push_back(one_edge);
            
            getline (input, line);
        }
}

void Prediction::displayPrediction(vector<string> list_of_states, vector<double> list_of_predictions){
    
    cv::Mat prediction_image = cv::Mat(cv::Size(((list_of_states.size()*70)+30), (300)), CV_8UC3);  
    prediction_image = cv::Scalar(0, 0, 0);
    
    
    for(auto count = 0; count < list_of_states.size(); count++){
	cv::rectangle(prediction_image, cv::Point(15+(count*70), 300), cv::Point((count*70)+45, 300.-(250.*list_of_predictions[count])), cv::Scalar(0, 0, 255), CV_FILLED);
    }
    
    cv::imshow("Prediction", prediction_image);
}

bool Prediction::interruptModule() {
    cout << "Interrupting your module, for port cleanup" << endl;
    return true;
}

void Prediction::sendActionCmd(string action_cmd, Point3d position_obj, Point3d target){
    Bottle action_cmd_bottle;
    
    action_cmd_bottle.addString("action");
    
    action_cmd_bottle.addString(action_cmd);
    
    action_cmd_bottle.addDouble(position_obj.x);
    action_cmd_bottle.addDouble(position_obj.y);
    action_cmd_bottle.addDouble(position_obj.z);
    
    if(target.x != 0){
        action_cmd_bottle.addDouble(target.x);
        action_cmd_bottle.addDouble(target.y);
        action_cmd_bottle.addDouble(target.z);
    }else{
        action_cmd_bottle.addDouble(position_obj.x);
        if(position_obj.y > 0){
            action_cmd_bottle.addDouble(position_obj.y-0.05);
        }else{
            action_cmd_bottle.addDouble(position_obj.y+0.05);
        }
        action_cmd_bottle.addDouble(position_obj.z);
    }
    
    portAck.write(action_cmd_bottle);
    
}

void Prediction::trainRobot(){
    string predicted_state, predicted_action;
    double p_S, p_S_A, p_S_A_S;
    Point3d position_of_target_object;
    
    vector<double> predictions;
    predictions.clear();
    
    double min_proba = INT_MAX; int min_index_state = 0;  int min_index_action = 0;
    bool eligible = false;
    
    int count = 0;
    if(!waiting_for_action_result && learning){
        for(auto state:current_states){
            if(state.find("STA") != std::string::npos){ //We train only on stationary objects
                for(auto action_j = 1; action_j < actions.size(); action_j++){
                    eligible = false;
                    
                    newEvidences(predicted_state, predicted_action, p_S, state, index_to_action[action_j]);
                    p_S_A = state0_to_action0(action_j, state_to_index[state]);
                    p_S_A_S = state0_and_action0_to_state1(state_to_index[predicted_state], action_j + actions.size()*state_to_index[state]);
                    
                    if(p_S > 0){
                        if((p_S_A <= training_limit(action_j)) || ((p_S_A_S < certainty) && (p_S_A_S >= 1-certainty))){
                            eligible = true;                        
                        }
                    }else{
                        if(p_S_A < training_limit(action_j)){
                            eligible = true;
                        }
                    }
                    
                    bool motion_possible = checkActionOnObject(current_states_position[count]);
                    if(eligible && p_S < min_proba && motion_possible){
                        min_proba = p_S;
                        min_index_state = state_to_index[state];
                        
                        position_of_target_object = current_states_position[count];
                        min_index_action = action_j;
                    }
                }
            }
            count++;
        }
        
        if(min_index_state != 0 && min_index_action != 0){
            cout<<"Perform "<<index_to_action[min_index_action]<<" on "<<index_to_state[min_index_state]<<endl;
            waiting_for_action_result = true;
            
            current_looked_at_state = min_index_state;
            current_perfomed_action = min_index_action;
            observed_related_states.clear();
            
            sendActionCmd(index_to_action[min_index_action], position_of_target_object);
            
        }
            
    }
}

bool Prediction::checkActionOnObject(Point3d position){
    Bottle actionTest;
    Bottle reply;
    bool doable;
    
    actionTest.addString("action");
    actionTest.addString("test_position");
    actionTest.addDouble(position.x);
    actionTest.addDouble(position.y);
    actionTest.addDouble(position.z);
    
    portAck.write(actionTest, reply);
    doable = reply.get(0).asBool();
    
    return doable;
}
    

bool Prediction::updateModule() {
    double distance_to_hand;
    
    int index_closest_object = -1;
    double min_distance = INT_MAX;
    bool hand_found = false;
    
    string predicted_state, predicted_action;
    double p_S;
    Point3d position_of_target_object;
    
    //Looking for others' hand 
    
    //If a hand is found, try to match it to a possible action the robot could do to help
    for(auto st_count = 0; st_count < current_states.size(); st_count++){
        distance_to_hand = sqrt((double)(pow((current_states_position[st_count].x - hand_position.x), 2) + pow((current_states_position[st_count].y - hand_position.y), 2)));
        if(distance_to_hand < min_distance){
            index_closest_object = st_count;
            min_distance = distance_to_hand;
            position_of_target_object = current_states_position[st_count];
        }        
    }
    if(index_closest_object != -1 && min_distance <= 0.1){
        hand_found = true;
    }
    
    //If their is no hand and if the robot is still learning, then train    
    if(!hand_found && learning){
        trainRobot();
    }else if(hand_found){
        newEvidences(predicted_state, predicted_action, p_S, current_states[index_closest_object], "NONE");
        if(predicted_action.compare("NONE") != 0 && predicted_state.compare("NONE") != 0){
            if(checkActionOnObject(current_states_position[index_closest_object])){
                waiting_for_action_result = true;
            
                current_looked_at_state = 0;
                current_perfomed_action = index_closest_object;
                observed_related_states.clear();
                
                sendActionCmd(predicted_action, position_of_target_object);
                
            }else{
                cout<<"Action not possible"<<endl;
            }
        }else{
            cout<<"No possible action to help"<<endl;
        }
    }
    
    cv::waitKey(1);
    
    return true;
}

bool Prediction::close() {
    cout << "Calling close function\n";
    return true;
}

double Prediction::getPeriod() {
    return 0.5;
}
