#include <iostream>
#include <vector>
using namespace std;

class Graph {
private:
    int V;
    vector<vector<int>> nodes; // {demand, priority, service_time}
    vector<vector<vector<int>>> adj; // {v, time, reliability}

public:
    Graph(int V) {
        this->V = V;
        adj.resize(V);
        nodes.resize(V, vector<int>(3, 0));
    }

    void addEdge(int u, int v, int time, int reliability) {
        adj[u].push_back({ v, time, reliability });
        adj[v].push_back({ u, time, reliability });
    }

    void deleteEdge(int u, int v) {
        // Remove from u’s adjacency list
        for (auto it = adj[u].begin(); it != adj[u].end(); it++) {
            if ((*it)[0] == v) {
                adj[u].erase(it);
                break;
            }
        }

        // Remove from v’s adjacency list
        for (auto it = adj[v].begin(); it != adj[v].end(); it++) {
            if ((*it)[0] == u) {
                adj[v].erase(it);
                break;
            }
        }
    }

    void printGraph() {
        for (int i = 0; i < V; i++) {
            cout << "Vertex " << i << ":";
            for (auto edge : adj[i]) {
                cout << " -> (" << edge[0]
                    << ", time=" << edge[1]
                    << ", rel=" << edge[2] << ")";
            }
            cout << endl;
        }
    }
};

//int main() {
//    Graph g(5);
//    g.addEdge(0, 1, 4, 90);
//    g.addEdge(0, 2, 6, 80);
//    g.addEdge(1, 2, 2, 70);
//    g.addEdge(1, 3, 5, 95);
//    g.addEdge(2, 3, 3, 85);
//    g.addEdge(3, 4, 4, 90);
//
//    cout << "Graph structure:\n";
//    g.printGraph();
//
//    cout << "\nDeleting edge (1,2)...\n";
//    g.deleteEdge(1, 2);
//
//    g.printGraph();
//
//    return 0;
//}
