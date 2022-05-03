#ifndef ASTAR_H
#define ASTAR_H

#include <string>
#include <vector>

class Astar
{
public:
    Astar(const std::vector<std::vector<int>> &grid, bool debug);
    Astar(const std::vector<std::vector<std::vector<int>>> &grid, bool debug);
    void setEdges(const std::vector<std::vector<int>> &grid, int i, int j);
    void setEdges(const std::vector<std::vector<std::vector<int>>> &grid, int i,
                  int j, int k);
    void printAdjMatrix();
    int convertTo1D(int i, int j);
    int convertTo1D(int i, int j, int k);
    std::vector<int> convertTo2D(int vertex);
    std::vector<int> convertTo3D(int vertex);
    std::vector<int> getAdjacent(int u);
    int minDistanceVertex(std::vector<int> distances, std::vector<bool> sptSet);
    int minDistanceAstar(std::vector<int> f, std::vector<int> open);
    std::vector<int> generateSuccessors(int u);
    std::vector<int> dijkstra(int u, int v);
    int heuristic(int u, int v, std::string metric);
    std::vector<int> astarPath(int u, int v);
    bool calculateAstarPath(int u, int v, std::vector<int> *parents);

private:
    int numVertices, numRows, numCols, numx, numy, numz;
    std::vector<std::vector<int>> adjMatrix;
    std::vector<std::vector<std::vector<int>>> adjMatrix3D;
};

#endif