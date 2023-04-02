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
    float score;
};

using cell = signed char;

const std::vector<int> dr_4 = {-1, 0, 0, 1};
const std::vector<int> dc_4 = {0, -1, 1, 0};

const std::vector<int> dr_8 = {-1, -1, -1, 0, 0, 1, 1, 1};
const std::vector<int> dc_8 = {-1, 0, 1, -1, 1, -1, 0, 1};

std::vector<cell> computeFrontierCellGrid(std::vector<cell> occupancyGrid, int width, int height);

std::vector<frontierRegion> computeFrontierRegions(std::vector<cell> frontierCellGrid, int width, int height,
    float resolution, float origin_x, float origin_y, int region_size_thresh);

std::vector<std::pair<int,int>> getNeighbours(int row, int col, int minRow, int minCol, int maxRow, int maxCol);

bool hasCellValue(std::vector<cell> grid, int width, std::vector<std::pair<int,int>> cells, int value);

frontierRegion selectFrontier(std::vector<frontierRegion>, int rank,
    float robot_pose_x, float robot_pose_y);

bool compareByScore(const frontierRegion &a, const frontierRegion &b);

#endif