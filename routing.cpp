#include <iostream>
#include <memory>
#include <optional> //for optional arg
#include <string>
#include <sstream>
#include <vector>
#include <map>
#include <utility>  // for std::pair
#include <climits> // For INT_MAX
#include <algorithm>
#include <fstream>
#include "json.hpp"

using namespace std;
using json = nlohmann::json;

void printVector(const vector<int> vec) {
    for (int num : vec) {
        cout << num << " ";
    }
    cout << endl;
}

using NodeID = int; // using er et alias for en anden klasse, så her er NodeID bare en int, men det er nice at have
using TrainID = int;

struct Route {
    bool possible;
    int distance;
    vector<NodeID> directions;

    void print () {
        if (!possible) {
            cout << "route not possible" << endl;
        }
        else {
            cout << "routing is ";
            for (int i = 0; i < directions.size(); i++) {
                cout << directions.at(i);
                if (i != directions.size() - 1) {   //if not last
                    cout << " -> ";
                } 
                else {
                    cout << endl;
                }
            }
            
        }
    }
};

enum NodeType {PLATFORM, HOLDING, CROSSING, SIGNAL, JUNCTION, UNKNOWN};
NodeType getNodeType(string type) {
    transform(type.begin(), type.end(), type.begin(), ::tolower);

    if (type == "platform") return PLATFORM;
    if (type == "holding")  return HOLDING;
    if (type == "crossing") return CROSSING;
    if (type == "signal")   return SIGNAL;
    if (type == "junction") return JUNCTION;
    else return UNKNOWN;
}

struct Edge {
    NodeID to;
    int distance;
    bool occupied = false;
};

class Node {
public:
    NodeID id; //nok dumt også at have her når ligger i key, men nice for sikkerhedsskyld
    string name;
    NodeType type;
    vector<Edge> edges;
    bool reversePossible;

    virtual bool checkTransition (NodeID from, NodeID to) { //er ikke safe, tjekker ikke om NodeID'erne er en del af edges
        if (reversePossible) {
            return true;
        } else if (from == to && reversePossible == false) {
            return false;
        } else {
            return true;
        }
    }
};

class PlatformNode: public Node {
    
};

class HoldingNode: public Node {

};

class CrossingNode: public Node {
    
};

class SignalNode: public Node {
    
};

class JunctionNode: public Node {
    public:
    vector<pair<NodeID,NodeID>> invalidTransitions; //from -> to

    bool checkTransition (NodeID from, NodeID to) override { //er ikke safe, tjekker ikke om NodeID'erne er en del af edges
        for (pair<NodeID,NodeID> invalidTransition : invalidTransitions) {
            if (invalidTransition.first == from && invalidTransition.second == to) {
                return false;
            }
        }
        if (!reversePossible && from == to) {
            return false;
        } else {
            return true;
        }
    }
};

using Graph = map<NodeID, shared_ptr<Node>>;

//make the graph
class TrackGraph {
    
    public:
        TrackGraph () { //init function
            cout << "TrackGraph initialised" << endl;
        }

        void loadTrackLayoutJSON (json layout) {
            //gå igennem nodes
            for(const auto& lNode : layout["nodes"]) {
                
                NodeType type = getNodeType(lNode["type"]);
                shared_ptr<Node> node;
        
                switch (type) {
                    case PLATFORM:
                    node = make_shared<PlatformNode>();
                    break;

                case HOLDING:
                    node = make_shared<HoldingNode>();
                    break;

                case CROSSING:
                    node = make_shared<CrossingNode>();
                    break;

                case SIGNAL:
                    node = make_shared<SignalNode>();
                    break;
                case JUNCTION: 
                    auto jNode = make_shared<JunctionNode>();
                    for (const auto lTransition : lNode["invalidTransitions"]) {
                        jNode->invalidTransitions.push_back(make_pair(lTransition["from"],lTransition["to"]));
                    }
                    node = jNode;
                    break;
                }
            
                //check om id valid
                if (graph_.find(lNode["id"]) != graph_.end()) {
                    NodeID nID = getNewNodeID();
                    cout << "node id invalid for id: " << lNode["id"] << ". Replacing with " << nID << endl;
                    node->id = nID;
                } else {
                    node->id = lNode["id"];
                }

                //give new name
                if (lNode["name"].size() == 1) {
                    cout << "name is empty lol, giving new birth with size: " << lNode["name"].size() << endl;
                    string typeStr = lNode["type"];
                    string frontCharStr = typeStr.substr(0,1);
                    cout << "frontchar: " << frontCharStr << endl;
                    string nName = frontCharStr + to_string(node->id);
                    node->name = nName;
                } else {
                    node->name = lNode["name"];
                    cout << "not giving new name with size: " << lNode["name"].size() << endl;
                }

                node->type = type;
                node->reversePossible = lNode["reversePossible"];
                node->edges = {};
                for (const auto lEdge : lNode["edges"]) {
                    Edge edge = Edge();
                    edge.to = lEdge["to"];
                    edge.distance = lEdge["distance"];
                    node->edges.push_back(edge);
                }

                graph_[node->id] = node;
            }
            cout << "track loaded from file" << endl;
        };

        void printGraph () {
            for (const auto& pair : graph_) {
                cout << pair.second->name << ":" << endl;
                for (int i=0; i < pair.second->edges.size(); i++) {
                    Edge current = pair.second->edges.at(i);
                    cout << "   -> " << graph_[current.to]->name << "    dist: " << current.distance << endl;
                }
            }
        }

        Route generateRoute(NodeID from, NodeID to) {

            map<pair<NodeID,NodeID>,pair<int,vector<NodeID>>> routes; //vi fylder ikke ud, men gør det undervejs i stedet for at lægge alt i med max int
            vector<pair<NodeID,NodeID>> unexplored;
            pair<NodeID,NodeID> current; //first = front, second = back

            //init
            current = make_pair(from,from);
            routes[current].first = 0;
            routes[current].second = {from};
            unexplored.push_back(current);


            while(true) {

                //check if we are there
                if(current.first == to) {
                    Route returnRoute;
                    returnRoute.possible = true;
                    returnRoute.distance = routes.at(current).first;
                    returnRoute.directions = routes.at(current).second;
                    return returnRoute;
                }

                //check edges
                for(Edge edge: graph_[current.first]->edges) {

                    //nu skal vi tjekke om routen existerer
                    bool validTransition = true;
                    bool routeAlreadyMapped = false;
                    pair<NodeID,NodeID> newNodePair = make_pair(edge.to,current.first);
                    int distanceForNewPair = routes.at(current).first + edge.distance;

                    //logic here
                    //check invalid trantions
                    validTransition = graph_[current.first]->checkTransition(current.second,edge.to);

                    //init for newly found nodes so distance can be max
                    if (routes.find(newNodePair) == routes.end()) {
                        routes[newNodePair] = {INT_MAX, {}};
                    }

                    //see if possible and if better than other
                    if(distanceForNewPair < routes[newNodePair].first && validTransition) {
                        //add route to routes
                        routes[newNodePair].first = distanceForNewPair;

                        vector<NodeID> newRoute = routes[current].second;
                        newRoute.push_back(edge.to);
                        routes[newNodePair].second = newRoute;

                        //add new find to unexplored
                        unexplored.push_back(newNodePair);
                    }
                }

                //remove from unexplord
                unexplored.erase(
                    remove(unexplored.begin(), unexplored.end(), current),
                    unexplored.end()
                );

                //check if there is any more land that has not been colonized
                if(unexplored.empty()) {
                    Route returnRoute;
                    returnRoute.possible = false;
                    return returnRoute;
                }

                //shift current
                pair<NodeID,NodeID> nextElement;

                nextElement = unexplored.at(0);

                for(pair<NodeID,NodeID> nodePair : unexplored) {
                    if (routes.at(nodePair).first < routes.at(nextElement).first) {
                        nextElement = nodePair;
                    }
                }

                current = nextElement;
            }
        }

        /* old route gen
        Route generateRouteOld_(NodeID from, NodeID to) { //complited, distnace, and route

            map<NodeID,pair<int,vector<NodeID>>> routes;
            vector<NodeID> unexplored;
            NodeID current = from;
            bool completed = false;

            for(const auto& pair : graph_) { //setup routes
                int distance = INT_MAX;
                if (pair.first == from) {
                    distance = 0;
                    routes[pair.first].second = {pair.first};    //vi sætter ruten til den første med sig selv
                }

                routes[pair.first].first = distance;
            }

            while(true) {

                for (Edge edge : graph_[current].edges) { //add new nodes to unexplored
                    int currentDistance = routes[current].first;
                    int totalDistance = currentDistance + edge.distance;
                    bool isValidTransition = true;

                    cout << "standing on " << current << " trying to rout to " << edge.to << endl;
                    cout << "   route is: ";
                    printVector(routes[current].second);

                    //check if reversing and possible
                    if (!graph_[current].reversePossible && edge.to == routes[current].second.at(routes[current].second.size() -1)) {
                        isValidTransition = false;
                    }

                    //check if trasition is valid for junction
                    if (routes[current].second.size() >= 2) {    //hvis der er mere end en node i routen tjekker vi om den forrige og kommende er en del af valid transitions
                        NodeID lastNode = routes[current].second[routes[current].second.size() -2];   //finder næst sidste node i routen som er den vi er kommet fra.

                        for (pair<NodeID,NodeID> invalidTransition : graph_[current].invalidTransitions) {

                            cout << "   checked if " << invalidTransition.first << " + " << invalidTransition.second << " == " << lastNode << " + " << edge.to << " while currrent node is " << current << endl;
                            

                            if (invalidTransition.first == lastNode && invalidTransition.second == edge.to) {   //hvis det er invalid transition skal vi stoppe:D
                                isValidTransition = false;
                            }
                        }
                    }

                    

                    if (totalDistance < routes[edge.to].first && isValidTransition) { //tjekker om der er kortere til end den fundne rute
                        vector<NodeID> newRoute = routes[current].second;   // vi laver ny rute som er identisk til den nuværende
                        newRoute.push_back(edge.to);    //vi lægger den nye node til ruten
                        pair<int,vector<NodeID>> routeInput = {totalDistance,newRoute};
                        routes[edge.to] = routeInput;
                        unexplored.push_back(edge.to);
                    }
                }

                if (unexplored.size() == 0) {
                    return {false,0,{}};
                }

                NodeID nextNode = unexplored.at(0); // shortest distance to
                
                for (NodeID id : unexplored) {  //find the shortest node
                    if (routes[id].first < routes[nextNode].first) {
                        nextNode = id;
                    }
                }

                if (nextNode == to) {
                    return {true, routes[nextNode].first, routes[nextNode].second}; //done
                }

                current = nextNode;
                unexplored.erase(remove(unexplored.begin(), unexplored.end(), nextNode), unexplored.end());

                
            }
            return {false,0,{}}; //return statement to make compiler happy
        }
            */

        Graph getGraph () {
            return graph_;
        }

    private:
        Graph graph_;

        NodeID getNewNodeID () {
            NodeID newID;
                if (graph_.empty()) {
                    newID = 0;
                }
                else {
                    NodeID lastID = graph_.rbegin()->first;
                    newID = lastID + 1;
                }
            return newID;
        }
};

struct Sensor {
    public:
        bool triggerd = false;

        void update () {
            if (recentReading != lastReading) {
                triggerd = true;
            }
        };
    
    private:
        bool recentReading = false;
        bool lastReading = false;

    void reset() {
        //wait 1 sec
        triggerd = false;
    };
};

class Train {
    public: 
        Train(TrainID trainID) { //init function
            id = trainID;
            cout << "train added with id: " << id << endl;
        }

        TrainID id;
        string name;
        int defaultSpeed;

        bool inRoute;
        Route route;
        
        //this is according to loaded route
        NodeID destination;
        NodeID startLocation;

        // this is according to actual location, no matter route
        NodeID nextNode;
        NodeID currentNode;

        void beginRoute () {

            //start train

            inRoute = true;
        }

        void onRouteCompletion () {}


    private:
        
};

class TrainManager {
    public:

        void addTrain (
            NodeID startingLocation,
            optional<string> name = nullopt,
            optional<int> defaultSpeed = nullopt
        ) {
            TrainID newTrainID = trains_.size();
            
            Train train(newTrainID);

            train.currentNode = startingLocation;

            if (name != nullopt) {
                train.name = name.value();
            } else {
                train.name = "train " + to_string(newTrainID);
            }

            if (defaultSpeed != nullopt) {
                train.defaultSpeed = defaultSpeed.value();
            } else {
                train.defaultSpeed = standardDefaultSpeed;
            }
        }

    private:

        map<TrainID,Train> trains_;

        int standardDefaultSpeed = 20;
};

bool checkIfIntInVector (vector<int> vector, int value) {
    for (int num : vector) {
        if (num == value) {
            return true;
        }
    }
    return false;
}

std::vector<std::string> splitString(const std::string& input, char delimiter) {    //chat generated
    std::vector<std::string> tokens;
    std::stringstream ss(input);
    std::string token;

    while (std::getline(ss, token, delimiter)) {
        tokens.push_back(token);
    }

    return tokens;
}

void loop () {

}

int main() {

    ifstream file("trackLayout.json");

    if (!file) {
        cerr << "Could not open trackLayout.json\n";
        return 1;
    }

    json layoutJSON;
    file >> layoutJSON;

    TrackGraph track;
    TrainManager trainManager;

    track.loadTrackLayoutJSON(layoutJSON);

    track.printGraph();

    return 0;
}