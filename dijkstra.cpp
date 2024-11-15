#include <bits/stdc++.h>

using namespace std;

// Structure to represent the edges of the graph: destination node and edge weight
struct Edge {
    int dest, weight;
};

// Structure to represent node-specific constraints (time, visit conditions, etc.)
struct NodeConstraint {
    int timeRequired;  // Minimum time at which this node can be visited
    // Additional fields for other constraints can be added here (e.g., fuelCost, etc.)
};

// Structure to represent the state during Dijkstra’s algorithm
struct State {
    int node, currentTime, cost;
    bool evenParity;  // Whether the path length is even or odd (for even-odd constraints)

    // Comparison operator to allow priority queue to sort by cost
    bool operator>(const State& other) const {
        return cost > other.cost;
    }
};

// Generalized Dijkstra’s function
int generalizedDijkstra(
    int n,                             // Number of nodes
    vector<vector<Edge>>& adjList,      // Adjacency list for the graph
    vector<NodeConstraint>& nodeConstraints,  // Node-specific constraints (e.g., time requirements)
    int start, int end)                 // Start and end nodes
    {
    vector<vector<int>> minCost(n, vector<int>(n, INT_MAX));  // To store the minimum cost to each node
    priority_queue<State, vector<State>, greater<State>> pq;

    // Initialize the priority queue with the starting node
    pq.push({start, 0, 0, true});  // Starting at time 0, cost 0, even parity
    minCost[start][0] = 0;

    while (!pq.empty()) {
        State current = pq.top();
        pq.pop();

        int node = current.node;
        int currTime = current.currentTime;
        int currCost = current.cost;
        bool parity = current.evenParity;

        // If the current node is the destination, return the cost
        if (node == end) return currCost;

        // ** Node-Specific Time Check **
        // Ensure that the node can be visited based on the current time
        if (currTime < nodeConstraints[node].timeRequired) {
            currTime = nodeConstraints[node].timeRequired;  // Set time to the minimum required
        }

        // ** Edge Traversal (to adjacent nodes) **
        for (Edge& edge : adjList[node]) {
            int nextNode = edge.dest;
            int travelCost = edge.weight;
            int newCost = currCost + travelCost;
            int arrivalTime = currTime + travelCost;  // Update time after traveling the edge

            // ** Node Time Requirement for Next Node **
            // Check if the arrival time meets the minimum time requirement for the next node
            if (arrivalTime < nodeConstraints[nextNode].timeRequired) continue;

            // ** Edge Parity Check (if applicable) **
            // Flip parity for even-odd edge count constraint problems
            bool newParity = !parity;

            // Update the cost and push the new state into the priority queue if it's better
            if (newCost < minCost[nextNode][arrivalTime]) {
                minCost[nextNode][arrivalTime] = newCost;
                pq.push({nextNode, arrivalTime, newCost, newParity});
            }
        }
    }

    return -1; // Return -1 if no valid path is found under the given constraints
}

int main() {
    int n = 5; // Number of nodes in the graph
    vector<vector<Edge>> adjList(n);

    // Node-specific constraints (e.g., time required to visit a node)
    vector<NodeConstraint> nodeConstraints(n);
    nodeConstraints[0] = {0};  // No restriction for node 0
    nodeConstraints[1] = {3};  // Node 1 can only be visited at or after time 3
    nodeConstraints[2] = {2};  // Node 2 can only be visited at or after time 2
    nodeConstraints[3] = {4};  // Node 3 can only be visited at or after time 4
    nodeConstraints[4] = {5};  // Node 4 can only be visited at or after time 5

    // Define the graph edges (node1 -> node2 with edge weight)
    adjList[0].push_back({1, 1}); // Edge from 0 to 1 with weight 1
    adjList[1].push_back({2, 2});
    adjList[2].push_back({3, 1});
    adjList[3].push_back({4, 3});

    int start = 0, end = 4;
    int minCost = generalizedDijkstra(n, adjList, nodeConstraints, start, end);

    if (minCost != -1) {
        cout << "Minimum cost to reach node " << end << " is: " << minCost << endl;
    } else {
        cout << "No valid path found under the given constraints." << endl;
    }

    return 0;
}
