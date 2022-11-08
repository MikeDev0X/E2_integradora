#include <iostream>
#include <vector>
#include <fstream>

using namespace std;


vector<vector<string>> readGraph(){
    string fileName = "graph.txt";
    


    ifstream graphFile(fileName.c_str());

        int N = 0;
        graphFile >> N;
        vector<vector<string>> graph(N, std::vector<string> (N));
        int iterador = 0;


        while(iterador < N){
            for(int x = 0; x < N; x++){
                graphFile >> graph [iterador][x];
            }
            iterador++;
        }
    
    return graph;
}

void printMatrix(vector<vector<string>> graph){
    for(int x = 0; x < graph.size(); x++){
        for(int y = 0; y < graph.size(); y++){
            cout << graph[x][y] << " ";
        }
        cout << endl;
    }
}



int main(){

    vector<vector<string>> graph = readGraph();
    printMatrix(graph);


    return EXIT_SUCCESS;
}


