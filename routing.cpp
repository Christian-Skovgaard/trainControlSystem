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

using NodeID = int; // using er et alias for en anden klasse, så her er NodeID bare en int, men det er nice at have
using Route = tuple<bool, int, vector<NodeID>>;

enum NodeType {PLATFORM, HOLDING, CROSSING, SIGNAL, JUNCTION};

struct Edge {
    NodeID to;
    int distance;
    bool occupied;
};

struct Node {
    NodeID id; //not dumt også at have her når ligger i key, men nice for sikkerhedsskyld
    string name;
    NodeType type;
    vector<Edge> edges;
    bool reversePossible;
    vector<pair<NodeID,NodeID>> invalidTransitions; //from -> to
};

using Graph = map<NodeID, Node>;

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
            Node node = Node();
        

            //name
            if (name != nullopt) {}
            else {
                switch (type) {
                    case PLATFORM: name = optional("Platform " + to_string(nodeID)); break;
                    case HOLDING: name = optional("Holdingtrack " + to_string(nodeID)); break;
                    case CROSSING: name = optional("Railcrossing " + to_string(nodeID)); break;
                    case SIGNAL: name = optional("Signal " + to_string(nodeID)); break;
                    case JUNCTION: name = optional("Junction " + to_string(nodeID)); break;
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

        Route generateRoute(NodeID from, NodeID to) { //complited, distnace, and route

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

                    if (routes[edge.to].second.size() > 1) {    //hvis der er mere end en node i routen tjekker vi om den forrige og kommende er en del af valid transitions
                        NodeID lastNode = routes[edge.to].second.at(routes[edge.to].second.size() -2);   //finder næst sidste node i routen som er den vi er kommet fra.
                        for (pair<NodeID,NodeID> invalidTransition : graph_[current].invalidTransitions) {
                            cout << "checked if " << invalidTransition.first << " + " << invalidTransition.second << " == " << lastNode << " + " << edge.to << "while currrent node is " << current << endl;
                            if (invalidTransition.first == lastNode && invalidTransition.second == edge.to) {   //hvis det er invalid transition skal vi stoppe:D
                                isValidTransition = false;
                            }
                        }
                    }
                    // NodeID previousNode = routes.at(-1)
                    // make invalid trasitions illigal

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

class sensor {
    public:
    bool triggerd = false;

    void reset() {
        //wait 1 sec
        triggerd = false;
    };
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
    track.addEdge(j3, j1, 80);
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

void printRoute(Route route) {
    //vi slår lige route i stykker, nemere at bruge:D
    bool success = get<0>(route);
    int distance = get<1>(route);
    vector<NodeID> directions = get<2>(route);

    if (!success) {
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



void loop () {

}

int main() {

    TrackGraph track;

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

        Route route = track.generateRoute(1,6);

        printRoute(route);

        done = true;

        

    }

    
    // cout << track.getGraph()[0].invalidTransitions.at(0).first << " + " << track.getGraph()[0].invalidTransitions.at(0).second << endl;

    //function that generates instructions upon feeding graph and points
    //instructions feed to train (via api to microcontroller)


    return 0;
}