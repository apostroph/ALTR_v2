/* 
 * Copyright (C): None
 * Authors: Jimmy Baraglia
 * Public License for more details
*/

#include "iCub/predictionUtils.hpp"

namespace predictionUtils
{
    void saveData(string src, vector<string> states, vector<string> actions, vector< vector<string> > edges){
        ofstream newSave;
        newSave.open ("../data/"+src+".xml");
        
        
        newSave << "<File>\n\n";
        
        newSave << "<StateSpace>\n";        
        for(auto state:states){
            newSave << state <<"\n";
        }        
        newSave << "</StateSpace>\n\n";
        
        newSave << "<ActionList>\n";        
        for(auto action:actions){
            newSave << action <<"\n";
        }        
        newSave << "</ActionList>\n\n";
        
        newSave << "<EdgeList>\n";        
        for(auto edge:edges){
            string element = edge[0]+","+edge[1]+","+edge[2];
            newSave << element <<"\n";
        }        
        newSave << "</EdgeList>\n\n";

        newSave << "</File>";
        newSave.close();
    }


}//End namespace