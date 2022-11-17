// Miguel Jiménez Padilla A01423189
// Marco Antonio Gardida Cortés A01423221

#include <iostream>
#include <vector>
#include <fstream>
#include <climits>
#include <queue>
#include <math.h>
#include <algorithm>

using namespace std;


///////////////// READ FILES ///////////////////////////

vector<vector<int>> readGraph(){
    string fileName = "graph.txt";

    ifstream graphFile(fileName.c_str());

        int N = 0;
        graphFile >> N;
        vector<vector<int>> graph(N, std::vector<int> (int(N)));
        int iterador = 0;

        string decoy = "";
        while(iterador < N){
            for(int x = 0; x < N; x++){
                graphFile >> graph[iterador][x];
            }
            iterador++;
        }

    return graph;
}

vector<vector<int>> readDataTrasmision(){
    string fileName = "data.txt";

    ifstream dataMatrix(fileName.c_str());

        int N = 0;
        dataMatrix >> N;
        vector<vector<int>> matrix(N, std::vector<int> (N));
        int iterador = 0;

        while(iterador < N){
            for(int x = 0; x < N; x++){
                dataMatrix >> matrix [iterador][x];
            }
            iterador++;
        }
    
    return matrix;
}

vector<vector<vector<int>>> readCentrales(){
    string fileName = "centrales.txt";

    ifstream centralesMatrix(fileName.c_str());

    int N, M;
    centralesMatrix >> N >> M;
    vector<vector<int>> centrales(M, vector<int> (N));
    vector<vector<int>> newPoint;

    vector<vector<vector<int>>> results;

    int iterador = 0;

    while(iterador < M){

        for(int f = 0; f < N; f++){
            centralesMatrix >> centrales[iterador][f];
        }
    iterador++;
    }


    int z, h;

    centralesMatrix >> z >> h;

    vector<int> newp;

    newp.push_back(z);
    newp.push_back(h);

    newPoint.push_back(newp);

    results.push_back(centrales); results.push_back(newPoint);
    return results;
}


void printMatrix(vector<vector<int>> graph){

    for(int x = 0; x < graph.size(); x++){
        for(int y = 0; y < graph[0].size(); y++){
            cout << graph[x][y] << " ";
        }
        cout << endl;
    }
}


///////////////////////////////////////////////

////////////////// PRIM //////////////////////

int minKey(vector<int> key, vector<bool> mstSet, int V)
{
  // Initialize min value
  int min = INT_MAX, min_index;
  for (int v = 0; v < V; v++)
  {
    if (mstSet[v] == false && key[v] < min)
    min = key[v], min_index = v;
  }
  return min_index;
}

void printMST(vector<int> parent, vector<vector<int>> graph, int printIndex)
{
  int count = 0;
  int first, last;

  first = parent[1];
  last = graph.size() - 1;

  cout << "Edge \tWeight\n";
  for (int i = 1; i < graph.size(); i++)
  {
    cout << parent[i] << " - " << i << " \t " << graph[i][parent[i]] << " \n";
    count += graph[i][parent[i]];
  }
  
  if(printIndex == 1){
    cout << last << " - " << first << " \t " << graph[first][last] << endl;
    count += graph[first][last];
  }
    cout<<"Total cost: "<<count<<endl<<endl;
}

// Function to construct and print MST for a graph represented using adjacency,  matrix representation
void primMST(vector<vector<int>> graph, int printIndex)
{
  // Array to store constructed MST
  vector<int> parent(graph.size());
  // Key values used to pick minimum weight edge in cut
  vector<int> key(graph.size());
  // To represent set of vertices included in MST
  vector<bool> mstSet(graph.size());
  // Initialize all keys as INFINITE
  for (int i = 0; i < graph.size(); i++){
    key[i] = INT_MAX;
    mstSet[i] = false;
  }
    
  int V = graph.size(); 
  
  // Always include first 1st vertex in MST,  Make key 0 so that this vertex is picked as first,  vertex.
  key[0] = 0;
  parent[0] = -1; // First node is always root of MST
  // The MST will have V vertices
  for (int count = 0; count < V - 1; count++) 
  {
    // Pick the minimum key vertex from the, set of vertices not yet included in MST
    int u = minKey(key, mstSet, V);
    // Add the picked vertex to the MST Set
    mstSet[u] = true;
    // Update key value and parent index of, the adjacent vertices of the picked vertex, Consider only those vertices which are not, yet included in MST
    for (int v = 0; v < V; v++)// graph[u][v] is nonzero only for adjacent
    // vertices of m mstSet[v] is false for vertices
    // not yet included in MST Update the key only
    // if graph[u][v] is smaller than key[v]
    if (graph[u][v] && mstSet[v] == false && graph[u][v] < key[v])
        {
            parent[v] = u;
            key[v] = graph[u][v];
        }
  }
  // print the constructed MST
  printMST(parent, graph, printIndex);
}

/////////////////////////////////////////////////////

/////////////// FORD FULKERSON /////////////////////

bool bfs(vector<vector <int>> rGraph, int s, int t, vector<int> &parent, int V)
{
    // Create a visited array and mark all vertices as not
    // visited
    vector<bool> visited (V);
    //    memset(visited, 0, sizeof(visited));

    //for(int f = 0; f < V; f++){
    //    visited[f] = false;
    //}

    fill(visited.begin(), visited.end(), 0);

 
    // Create a queue, enqueue source vertex and mark source
    // vertex as visited
    queue<int> q;
    q.push(s);
    visited[s] = true;
    parent[s] = -1;
 
    // Standard BFS Loop
    while (!q.empty()) {
        int u = q.front();
        q.pop();
    
        for (int v = 0; v < V; v++) {
            if (visited[v] == false && rGraph[u][v] > 0) {
                // If we find a connection to the sink node,
                // then there is no point in BFS anymore We
                // just have to set its parent and can return
                // true
                if (v == t) {
                    parent[v] = u;
                    return true;
                }
                q.push(v);
                parent[v] = u;
                visited[v] = true;
            }
        }
    }
 
    // We didn't reach sink in BFS starting from source, so
    // return false
    return false;
}

int fordFulkerson(vector<vector<int>> graph, int s, int t, int V)
{
    int u, v;
 
    // Create a residual graph and fill the residual graph
    // with given capacities in the original graph as
    // residual capacities in residual graph
    //int rGraph[V]
              //[V]; // Residual graph where rGraph[i][j]
                   // indicates residual capacity of edge
                   // from i to j (if there is an edge. If
                   // rGraph[i][j] is 0, then there is not)

    vector<vector<int>> rGraph (V, std::vector<int> (V,0));

    for (u = 0; u < V; u++)
        for (v = 0; v < V; v++)
            rGraph[u][v] = graph[u][v];
 
    vector<int> parent(V); // This array is filled by BFS and to
                   // store path
 
    int max_flow = 0; // There is no flow initially
 
    // Augment the flow while there is path from source to
    // sink
    while (bfs(rGraph, s, t, parent, V)) {
        // Find minimum residual capacity of the edges along
        // the path filled by BFS. Or we can say find the
        // maximum flow through the path found.
        int path_flow = INT_MAX;
        for (v = t; v != s; v = parent[v]) {
   

            u = parent[v];
            path_flow = min(path_flow, rGraph[u][v]);
        }
 
        // update residual capacities of the edges and
        // reverse edges along the path
        for (v = t; v != s; v = parent[v]) {
            u = parent[v];
            rGraph[u][v] -= path_flow;
            rGraph[v][u] += path_flow;
        }
 
        // Add path flow to overall flow
        max_flow += path_flow;
    }
 
    // Return the overall flow
    return max_flow;
}


////////////////////////////////////////////////////////////

///////////////////// MIN DISTANCE ////////////////////////

int minDistance(int centralsNum, vector<vector<int>> centralsMatrix, vector<int> newPoint){
    vector<int> distances;
    
    int distance = 0;
    int result = 0;

    int x2 = newPoint[0];
    int y2 = newPoint[1];


    for (int x = 0; x < centralsNum; x++){
        int prim = centralsMatrix[x][0];
        int sec = centralsMatrix[x][1];

        distance = sqrt(pow(x2 - prim, 2) + pow(y2 - sec,2));
        distances.push_back(distance);

    }

    int minvalue = *min_element(distances.begin(), distances.end());


    return minvalue;

}


int main(){

    vector<vector<int>> graph = readGraph();
    vector<vector<int>> dataTransmission = readDataTrasmision();
    vector<vector<int>> centrales = readCentrales()[0];
    vector<int> newPoint = readCentrales()[1][0];

    cout << "┌─────────────────────────┐" << endl;
    cout << "│           DATA          │" << endl;
    cout << "└─────────────────────────┘" << endl;

    cout << endl;

    cout  << "┌─────────────────┐"<< endl;
    cout  << "- Adjacency Graph:- " << endl;
    cout  << "└─────────────────┘" << endl;


    printMatrix(graph);
    cout << endl;

    cout  << "┌─────────────────────────────────┐"<< endl;
    cout  << "- Max data transmission capacity: - " << endl;
    cout  << "└─────────────────────────────────┘" << endl;    printMatrix(dataTransmission);
    cout << endl;

    cout  << "┌─────────────────┐"<< endl;
    cout  << "-    Centrals:-    " << endl;
    cout  << "└─────────────────┘" << endl;

    printMatrix(centrales);
    cout << endl;

    cout  << "┌──────────────────┐"<< endl;
    cout  << "- New point to add:- " << endl;
    cout  << "└──────────────────┘" << endl;    
    cout << newPoint[0] << " " << newPoint[1] << endl << endl;


    cout << "┌─────────────────────────┐" << endl;
    cout << "│          PART 1         │" << endl;
    cout << "└─────────────────────────┘" << endl;
    
    // Parte 1
    primMST(graph,0);

    
    cout << "┌─────────────────────────┐" << endl;
    cout << "│          PART 2         │" << endl;
    cout << "└─────────────────────────┘" << endl;

    // Parte 2
    primMST(graph,1);

    
    cout << "┌─────────────────────────┐" << endl;
    cout << "│          PART 3         │" << endl;
    cout << "└─────────────────────────┘" << endl;
    // Parte 3
    cout << "The max flow is: " << fordFulkerson(dataTransmission, 0, dataTransmission.size() - 1, dataTransmission.size()) << endl;


    
    cout << "┌─────────────────────────┐" << endl;
    cout << "│          PART 4         │" << endl;
    cout << "└─────────────────────────┘" << endl;
    // Parte 4
    cout << "The shortest distance between nodes is: " << minDistance(centrales.size(), centrales, newPoint) << endl << endl;

    return EXIT_SUCCESS;
}