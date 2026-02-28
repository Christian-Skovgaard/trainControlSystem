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
using namespace std;

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

enum NodeType {PLATFORM, HOLDING, CROSSING, SIGNAL, JUNCTION};

struct Edge {
    NodeID to;
    int distance;
    bool occupied;
};

class Node {
    NodeID id; //nok dumt også at have her når ligger i key, men nice for sikkerhedsskyld
    string name;
    NodeType type;
    vector<Edge> edges;
    bool reversePossible;
    vector<pair<NodeID,NodeID>> invalidTransitions; //from -> to
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

};

using Graph = map<NodeID, shared_ptr<Node>>;

//make the graph
class TrackGraph {
    
    public:
        TrackGraph () { //init function
            cout << "TrackGraph initialised" << endl;
        }

        NodeID addNode (
            NodeType type, 
            optional<string> name = nullopt, 
            optional<bool> reversePossible = nullopt,
            optional<vector<pair<NodeID,NodeID>>> invalidTransitions = nullopt
        ) {
            NodeID nodeID = getNewNodeID();
            Node node;
        

            //name
            if (name != nullopt) {}
            else {
                switch (type) {
                    case PLATFORM: 
                        name = optional("Platform " + to_string(nodeID));
                        node = PlatformNode();
                        break;
                    case HOLDING: 
                        name = optional("Holdingtrack " + to_string(nodeID)); 
                        node = HoldingNode();
                        break;
                    case CROSSING: 
                        name = optional("Railcrossing " + to_string(nodeID)); 
                        node = CrossingNode();
                        break;
                    case SIGNAL: 
                        name = optional("Signal " + to_string(nodeID)); 
                        node = SignalNode();
                        break;
                    case JUNCTION: 
                        name = optional("Junction " + to_string(nodeID)); 
                        node = JunctionNode();
                        break;
                }
            }

            //reversePossible
            if (reversePossible != nullopt) {}
            else if (type == PLATFORM || type == HOLDING) {reversePossible = optional(true);}
            else {reversePossible = optional(false);}

            node.id = nodeID;
            node.type = type;
            node.name = name.value();
            node.reversePossible = reversePossible.value();
            if (invalidTransitions != nullopt) {node.invalidTransitions = invalidTransitions.value();}

            graph_[nodeID] = node;

            cout << "node added with id: " << nodeID << " as type: " << type << endl;

            return nodeID;
        }

        void loadTrackLayoutJSON () {}

        void addEdge (NodeID from, NodeID to, float distance) {
            Edge edge = Edge();
            edge.distance = distance;

            edge.to = to;
            graph_[from].edges.push_back(edge);

            edge.to = from;
            graph_[to].edges.push_back(edge);

            cout << "edge added going from: " << from << " -> " << to << endl;
        }

        void addInvalidTransitions (NodeID nodeID, vector<pair<NodeID,NodeID>> list) {
            for (const auto& pair : list) {
                graph_[nodeID].invalidTransitions.push_back(pair);
            }
        };

        void printGraph () {
            for (const auto& pair : graph_) {
                cout << pair.second.name << ":" << endl;
                for (int i=0; i < pair.second.edges.size(); i++) {
                    Edge current = pair.second.edges.at(i);
                    cout << "   -> " << graph_[current.to].name << "    dist: " << current.distance << endl;
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
                for(Edge edge: graph_[current.first].edges) {

                    //nu skal vi tjekke om routen existerer
                    bool validTransition = true;
                    bool routeAlreadyMapped = false;
                    pair<NodeID,NodeID> newNodePair = make_pair(edge.to,current.first);
                    int distanceForNewPair = routes.at(current).first + edge.distance;

                    //logic here
                    //check invalid trantions
                    for (pair<NodeID,NodeID> invalidTranstion : graph_.at(current.first).invalidTransitions) {
                        pair<NodeID,NodeID> transition = make_pair(current.second,edge.to);
                        if (transition == invalidTranstion) {
                            validTransition = false;
                        }
                    }

                    //check if is reversing without consent
                    if(!graph_.at(current.first).reversePossible && current.second == edge.to) {
                        validTransition = false;
                    }

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

void createLayout(TrackGraph& track) {
    int j1 = track.addNode(JUNCTION);
    int p1 = track.addNode(PLATFORM,"Struer st. spor 1", true);
    int p2 = track.addNode(PLATFORM,"Struer st. spor 2", true);
    int j2 = track.addNode(JUNCTION);
    int c1 = track.addNode(CROSSING);
    int j3 = track.addNode(JUNCTION);
    int h1 = track.addNode(HOLDING);

    track.addEdge(j1, p1, 20);
    track.addEdge(j1, p2, 20);
    track.addInvalidTransitions(j1,{{p1,p2},{p2,p1}});
    track.addEdge(p1, j2, 20);
    track.addEdge(p2, j2, 20);
    track.addInvalidTransitions(j2,{{p1,p2},{p2,p1}});
    track.addEdge(j2, c1, 40);
    track.addEdge(c1, j3, 40);
    track.addEdge(j3, h1, 20);
    track.addEdge(j3, j1, 15);
    track.addInvalidTransitions(j3,{{h1,j1},{j1,h1}});

    cout << "Track layout inserted" << endl;
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

    TrackGraph track;
    TrainManager trainManager;

    createLayout(track);

    // track.printGraph();

    bool done = false;

    while(!done) {

        string input;

        //cin.ignore();
        //getline(cin,input);

        if(input == "done") {
            done = true;
        }

        //vector<string> params = splitString(input,' ');

        //cout << "params is: " << params.at(0) << " and " << params.at(1) << endl;

        //Route route = track.generateRoute(stoi(params.at(0)),stoi(params.at(1)));

        cout << "routing from 1 to 6" << endl;
        Route route = track.generateRoute(2,1);

        route.print();

        done = true;

        

    }

    // cout << track.getGraph()[0].invalidTransitions.at(0).first << " + " << track.getGraph()[0].invalidTransitions.at(0).second << endl;

    //function that generates instructions upon feeding graph and points
    //instructions feed to train (via api to microcontroller)


    return 0;
}