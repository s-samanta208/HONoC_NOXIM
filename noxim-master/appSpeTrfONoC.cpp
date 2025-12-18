// command to generate table
// g++ tableCreate.cpp
// ./a.out <Graph_no>
// mapping and graph data file should be in src/mapping_data folder
// output generated in src/tables folder

#include <iostream>
#include <fstream>
#include <vector>
#include <climits>
#include <cstring>
using namespace std;

const int INF = INT_MAX; // Define INF value as maximum integer value
int noNodes = 0;
struct Edge {
    int from, to;
    double weight;
    Edge(int f, int t, double w) : from(f), to(t), weight(w) {}
};

vector<Edge> readGraph(const string& filename) {
    ifstream file(filename);
    if (!file.is_open()) {
        cerr << "Error: Unable to open file " << filename << endl;
        exit(1);
    }

    int n;
    file >> n;
    noNodes = n;
    vector<Edge> edges;
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n; ++j) {
            string buff;
            file >> buff;
            if (buff != "INF" && j>i) {
                edges.push_back(Edge(i, j, stod(buff)));
            }
        }
    }

    file.close();
    return edges;
}

vector<int> readMapping(const string& filename) {
    ifstream file(filename);
    if (!file.is_open()) {
        cerr << "Error: Unable to open file " << filename << endl;
        exit(1);
    }

    string s;
    file >> s;
    file >> s;
    vector<int> mapping;
    for(int i = 0;i<noNodes;i++){
        int n;
        file >> n;
        mapping.push_back(n);
    }

    for(int i = 0;i<noNodes;i++) cout<<mapping[i]<<" ";
    file.close();
    return mapping;
}

int main(int argc,char* argv[]) {
    char* filename = (char*)malloc(1024*sizeof(char));
    strcpy(filename,"mapping_data/Graph");
    strcat(filename,argv[1]);
    strcat(filename,".txt"); // Change the filename as needed   
    char* fname1 = (char*)malloc(1024*sizeof(char));
    strcpy(fname1,"mapping_data/out_Graph");
    strcat(fname1,argv[1]);
    strcat(fname1,".txt");
    vector<Edge> edges = readGraph(filename);
    // cout<<edges.size();
    vector<int> mapping = readMapping(fname1);
    // Print the edges
    // cout << "Edges in the graph:" << endl;
    
    double maxWt = -1;
    for(const auto& edge:edges){
        if(maxWt<edge.weight) maxWt = edge.weight;
    }
    string fname = "tables/outGraph";
    fname+=argv[1];
    fname+=".txt";
    ofstream file(fname);
    if (!file.is_open()) {
        cerr << "Error: Unable to open file " << filename << endl;
        exit(1);
    }

    for (const auto& edge : edges) {
        int u = edge.from;
        int v = edge.to;
        double wt = edge.weight;
        int u_i = -1;
        int v_i = -1;
        for(int i = 0;i<noNodes;i++){
            if(mapping[i] == u) u_i = i;
            if(mapping[i] == v) v_i = i;
        }
        // cout<<wt<<endl;
        wt = wt/maxWt;
        if(u_i == v_i) continue;
        if(u_i != -1 && v_i != -1)
        file<<u_i<<" "<<v_i<<" "<<wt<<endl;
    }   
    file.close();
    return 0;
}
