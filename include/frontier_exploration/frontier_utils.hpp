#ifndef FRONTIER_UTILS_
#define FRONTIER_UTILS_

#include<iostream>
#include<vector>
#include<utility>
#include<queue>

struct frontierRegion {
    int size;
    float x;
    float y;
};

using cell = signed char;

std::vector<int> dr_4 = {-1, 0, 0, 1};
std::vector<int> dc_4 = {0, -1, 1, 0};

std::vector<int> dr_8 = {-1, -1, -1, 0, 0, 1, 1, 1};
std::vector<int> dc_8 = {-1, 0, 1, -1, 1, -1, 0, 1};

std::vector<int> computeFrontierCellGrid(std::vector<int> occupancyGrid, int width, int height);

std::vector<frontierRegion> computeFrontierRegions(std::vector<int> frontierCellGrid, int width, int height);

std::vector<std::pair<int,int>> getNeighbours(int row, int col, int minRow, int minCol, int maxRow, int maxCol);

bool hasCellValue(std::vector<int> grid, int width, std::vector<std::pair<int,int>> cells, int value);

#endif