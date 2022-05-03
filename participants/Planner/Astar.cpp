#include "Astar.h"
#include <algorithm>
#include <assert.h>
#include <cmath>
#include <iostream>
#include <limits.h>
#include <math.h>

Astar::Astar(const std::vector<std::vector<int>> &grid, bool debug)
{
    assert(grid.size() > 0);
    this->numRows = grid.size();
    this->numCols = grid[0].size();
    this->numVertices = this->numRows * this->numCols;

    std::vector<std::vector<int>> vec(this->numVertices,
                                      std::vector<int>(this->numVertices, 0));
    this->adjMatrix = vec;

    for (int i = 0, n = grid.size(); i < n; ++i)
        for (int j = 0, m = grid[i].size(); j < m; ++j)
            setEdges(grid, i, j);

    if (debug)
    {
        std::cout << "Adjacency Matrix: " << std::endl;
        printAdjMatrix();
    }
}

Astar::Astar(const std::vector<std::vector<std::vector<int>>> &grid, bool debug)
{
    assert(grid.size() > 0);
    this->numx = grid.size();
    this->numy = grid[0].size();
    this->numz = grid[0][0].size();
    this->numVertices = this->numx * this->numy * this->numz;

    std::vector<std::vector<int>> vec(this->numVertices,
                                      std::vector<int>(this->numVertices, 0));
    this->adjMatrix = vec;

    for (int i = 0, n = grid.size(); i < n; ++i)
        for (int j = 0, m = grid[i].size(); j < m; ++j)
            for (int k = 0, o = grid[i][j].size(); k < o; ++k)
                setEdges(grid, i, j, k);
}

void Astar::printAdjMatrix()
{
    for (int i = 0, n = this->numVertices; i < n; ++i)
    {
        for (int j = 0; j < n; ++j)
            std::cout << this->adjMatrix[i][j] << " ";
        std::cout << std::endl;
    }
}

int Astar::convertTo1D(int i, int j) { return i * this->numCols + j; }

int Astar::convertTo1D(int i, int j, int k)
{
    return i * this->numy * this->numz + j * this->numz + k;
}

std::vector<int> Astar::convertTo2D(int vertex)
{
    std::vector<int> result;
    int x = vertex / this->numCols, y = vertex % this->numCols;
    result.push_back(x);
    result.push_back(y);
    return result;
}

std::vector<int> Astar::convertTo3D(int vertex)
{
    int x = vertex / (this->numy * this->numz),
        y = (vertex / (this->numz)) % this->numy, z = vertex % this->numz;
    return {x, y, z};
}

void Astar::setEdges(const std::vector<std::vector<std::vector<int>>> &grid,
                     int i, int j, int k)
{
    if (grid[i][j][k] == 1)
        return;
    if (i > 0 && grid[i - 1][j][k] != 1)
    {
        int u = convertTo1D(i, j, k), v = convertTo1D(i - 1, j, k);
        this->adjMatrix[u][v] = 1;
        this->adjMatrix[v][u] = 1;
    }
    if (i < numx - 1 && grid[i + 1][j][k] != 1)
    {
        int u = convertTo1D(i, j, k), v = convertTo1D(i + 1, j, k);
        this->adjMatrix[u][v] = 1;
        this->adjMatrix[v][u] = 1;
    }
    if (j > 0 && grid[i][j - 1][k] != 1)
    {
        int u = convertTo1D(i, j, k), v = convertTo1D(i, j - 1, k);
        this->adjMatrix[u][v] = 1;
        this->adjMatrix[v][u] = 1;
    }
    if (j < numy - 1 && grid[i][j + 1][k] != 1)
    {
        int u = convertTo1D(i, j, k), v = convertTo1D(i, j + 1, k);
        this->adjMatrix[u][v] = 1;
        this->adjMatrix[v][u] = 1;
    }
    if (k > 0 && grid[i][j][k - 1] != 1)
    {
        int u = convertTo1D(i, j, k), v = convertTo1D(i, j, k - 1);
        this->adjMatrix[u][v] = 1;
        this->adjMatrix[v][u] = 1;
    }
    if (k < numz - 1 && grid[i][j][k + 1] != 1)
    {
        int u = convertTo1D(i, j, k), v = convertTo1D(i, j, k + 1);
        this->adjMatrix[u][v] = 1;
        this->adjMatrix[v][u] = 1;
    }
}

void Astar::setEdges(const std::vector<std::vector<int>> &grid, int i, int j)
{
    if (grid[i][j] == 1)
        return;
    if (i > 0 && grid[i - 1][j] != 1)
    {
        int u = convertTo1D(i, j), v = convertTo1D(i - 1, j);
        this->adjMatrix[u][v] = 1;
        this->adjMatrix[v][u] = 1;
        if (j > 0 && grid[i - 1][j - 1] != 1)
        {
            v = convertTo1D(i - 1, j - 1);
            this->adjMatrix[u][v] = 1;
            this->adjMatrix[v][u] = 1;
        }
        else if (j < numCols - 1 && grid[i - 1][j + 1] != 1)
        {
            v = convertTo1D(i - 1, j + 1);
            this->adjMatrix[u][v] = 1;
            this->adjMatrix[v][u] = 1;
        }
    }
    if (j > 0 && grid[i][j - 1] != 1)
    {
        int u = convertTo1D(i, j), v = convertTo1D(i, j - 1);
        this->adjMatrix[u][v] = 1;
        this->adjMatrix[v][u] = 1;
    }
    if (i < numRows - 1 && grid[i + 1][j] != 1)
    {
        int u = convertTo1D(i, j), v = convertTo1D(i + 1, j);
        this->adjMatrix[u][v] = 1;
        this->adjMatrix[v][u] = 1;
        if (j > 0 && grid[i + 1][j - 1] != 1)
        {
            v = convertTo1D(i + 1, j - 1);
            this->adjMatrix[u][v] = 1;
            this->adjMatrix[v][u] = 1;
        }
        else if (j < numCols - 1 && grid[i + 1][j + 1] != 1)
        {
            v = convertTo1D(i + 1, j + 1);
            this->adjMatrix[u][v] = 1;
            this->adjMatrix[v][u] = 1;
        }
    }
    if (j < numCols - 1 && grid[i][j + 1] != 1)
    {
        int u = convertTo1D(i, j), v = convertTo1D(i, j + 1);
        this->adjMatrix[u][v] = 1;
        this->adjMatrix[v][u] = 1;
    }
}

std::vector<int> Astar::getAdjacent(int u)
{
    std::vector<int> result;

    for (int v = 0; v < this->numVertices; ++v)
        if (this->adjMatrix[u][v] != 0)
            result.push_back(v);

    return result;
}

int Astar::minDistanceVertex(std::vector<int> distances,
                             std::vector<bool> visited)
{
    int min = INT_MAX, index = 0;

    for (int v = 0; v < this->numVertices; ++v)
    {
        if (!visited[v] && distances[v] < min)
        {
            min = distances[v];
            index = v;
        }
    }

    return index;
}

std::vector<int> Astar::dijkstra(int u, int v)
{
    std::vector<int> distances(this->numVertices, INT_MAX);
    std::vector<bool> visited(this->numVertices, false);
    std::vector<int> parents(this->numVertices, -1);
    std::vector<int> result;

    distances[u] = 0;

    for (int i = 0; i < this->numVertices; ++i)
    {
        int j = minDistanceVertex(distances, visited);

        visited[j] = true;

        for (int k = 0; k < this->numVertices; ++k)
        {
            if (!visited[k] && this->adjMatrix[j][k] != 0 &&
                distances[j] != INT_MAX)
            {
                if (distances[j] + this->adjMatrix[j][k] < distances[k])
                {
                    distances[k] = distances[j] + this->adjMatrix[j][k];
                    parents[k] = j;
                }
            }
        }
    }

    if (parents[v] != -1)
        for (int i = v; i != u; i = parents[i])
            result.push_back(i);

    std::reverse(result.begin(), result.end());
    return result;
}

int Astar::minDistanceAstar(std::vector<int> f, std::vector<int> open)
{
    int min = INT_MAX, index = 0;

    for (int v = 0, n = open.size(); v < n; ++v)
    {
        if (f[v] < min)
        {
            min = f[v];
            index = v;
        }
    }

    return index;
}

std::vector<int> Astar::generateSuccessors(int u)
{
    std::vector<int> successors;
    for (int i = 0; i < this->numVertices; ++i)
        if (this->adjMatrix[u][i] != 0)
            successors.push_back(i);
    return successors;
}

std::vector<int> Astar::astarPath(int u, int v)
{
    std::vector<int> parents(this->numVertices, -1);
    std::vector<int> result;

    bool worked = calculateAstarPath(u, v, &parents);
    std::cout << worked << std::endl;

    if (parents[v] != -1)
        for (int i = v; i != u; i = parents[i])
            result.push_back(i);

    std::reverse(result.begin(), result.end());
    return result;
}

int Astar::heuristic(int u, int v, std::string metric)
{
    std::vector<int> u_coords = convertTo2D(u), v_coords = convertTo2D(v);
    int result = INT_MAX;

    if (metric == "euclidean")
        result = sqrt(pow(v_coords[0] - u_coords[0], 2) +
                      pow(v_coords[1] - u_coords[1], 2));
    else if (metric == "manhattan")
        result = abs(v_coords[0] - u_coords[0]) + abs(v_coords[1] - u_coords[1]);
    else if (metric == "octagonal")
        result = 10 * (abs(v_coords[0] - u_coords[0]) + abs(v_coords[1] - u_coords[1])) - 6 * std::min(abs(v_coords[0] - u_coords[0]), abs(v_coords[1] - u_coords[1]));
    else
        result = abs(v_coords[0] - u_coords[0]) + abs(v_coords[1] - u_coords[1]);

    return result;
}

bool Astar::calculateAstarPath(int u, int v, std::vector<int> *parents)
{
    std::vector<int> f(this->numVertices, INT_MAX);
    std::vector<int> g(this->numVertices, INT_MAX);
    std::vector<int> h(this->numVertices, INT_MAX);
    std::vector<int> open;
    std::vector<int> closed;

    open.push_back(u);

    f[u] = 0;
    g[u] = 0;
    h[u] = 0;

    while (open.size() > 0)
    {
        int minOpenIndex = minDistanceAstar(f, open);
        int minVertex = open[minOpenIndex];

        open.erase(open.begin() + minOpenIndex);

        std::vector<int> successors = generateSuccessors(minVertex);

        for (int k = 0, n = successors.size(); k < n; ++k)
        {
            int successor = successors[k];

            if (successor == v)
            {
                (*parents)[successor] = minVertex;
                return true;
            }

            int g_new = g[minVertex] + this->adjMatrix[minVertex][successor];
            int h_new = heuristic(successor, v, "manhattan");
            int f_new = g[successor] + h[successor];

            if (f[successor] == INT_MAX || f_new < f[successor])
            {
                open.push_back(successor);
                f[successor] = f_new;
                g[successor] = g_new;
                h[successor] = h_new;
                (*parents)[successor] = minVertex;
            }
        }
        closed.push_back(minVertex);
    }

    return false;
}

/*
int gridx_size = 4, gridy_size = 4, gridz_size = 4;
using namespace std::chrono;

// TODO: floor might not be imported yet, test
std::vector<int> convertPositionToGrid(std::vector<float> grid_start, std::vector<float> grid_end, std::vector<float> point)
{
    float x_0 = grid_start[0], x_1 = grid_end[0], y_0 = grid_start[1], y_1 = grid_end[1], z_0 = grid_start[2], z_1 = grid_end[2];
    float step_x = (x_1 - x_0) / gridx_size, step_y = (y_1 - y_0) / gridy_size, step_z = (z_1 - z_0) / gridz_size;
    return {(int)floor((point[0] + 0.01 - x_0) / step_x), (int)floor((point[1] + 0.01 - y_0) / step_y), (int)floor((point[2] + 0.01 - z_0) / step_z)};
}

std::vector<float> convertGridToPosition(std::vector<float> grid_start, std::vector<float> grid_end, std::vector<int> point)
{
    float x_0 = grid_start[0], x_1 = grid_end[0], y_0 = grid_start[1], y_1 = grid_end[1], z_0 = grid_start[2], z_1 = grid_end[2];
    float step_x = (x_1 - x_0) / gridx_size, step_y = (y_1 - y_0) / gridy_size, step_z = (z_1 - z_0) / gridz_size;
    return {x_0 + point[0] * step_x, y_0 + point[1] * step_y, z_0 + point[2] * step_z};
}

int pointToVertex(const std::vector<int> &point)
{
    return point[0] * gridy_size + point[1];
}

int pointToVertex3D(const std::vector<int> &point)
{
    return point[0] * gridy_size * gridz_size + point[1] * gridz_size + point[2];
}

std::vector<int> vertexToPoint(int vertex)
{
    std::vector<int> result = {vertex / gridSize, vertex % gridSize};
    return result;
}

std::vector<int> vertexToPoint3D(int vertex)
{
    std::vector<int> result = {vertex / (gridz_size * gridy_size), (vertex / gridz_size) % gridy_size, vertex % gridz_size};
    return result;
}

/// TODO assert if drone is in blocked position
void initializeGrid(const std::vector<std::vector<int>> &points,
                    std::vector<std::vector<int>> &grid)
{
    for (int i = 0; i < gridx_size; ++i)
    {
        std::vector<int> row;
        for (int j = 0; j < gridy_size; ++j)
        {
            row.push_back(0);
        }
        grid.push_back(row);
    }

    for (int i = 0, n = points.size(); i < n; ++i)
    {
        int x = points[i][0];
        int y = points[i][1];
        grid[x][y] = 1;
    }
}

void initializeGrid(const std::vector<std::vector<int>> &points,
                    std::vector<std::vector<std::vector<int>>> &grid)
{
    // initialize grid
    for (int i = 0; i < gridx_size; ++i)
    {
        std::vector<std::vector<int>> row;
        for (int j = 0; j < gridy_size; ++j)
        {
            std::vector<int> height;
            for (int k = 0; k < gridz_size; ++k)
            {
                height.push_back(0);
            }
            row.push_back(height);
        }
        grid.push_back(row);
    }

    for (int i = 0, n = points.size(); i < n; ++i)
    {
        int x = points[i][0], y = points[i][1], z = points[i][2];
        grid[x][y][z] = 1;
    }
}
*/