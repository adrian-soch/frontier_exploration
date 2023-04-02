#include "frontier_exploration/frontier_utils.hpp"

using namespace std;

/**
 * This function prints a one-dimensional list of cell values as a grid
 *
 * @param	inputVector	the input vector representing the grid contents
 * @param	width		the grid width
 * @param	height		the grid height
 */
void printGrid(vector<int> inputVector, int width, int height){
    
    cout << width << " " << height << endl;
    for (int i = 0; i < height; i++){
        for(int j = 0; j < width; j++){
            cout << inputVector[i*width +j] << "\t";
        }
        cout << endl;
    }
}

/**
 * This function prints a list of frontier regions
 *
 * @param	frontierRegions		the input list of regions
 */
void printFrontierRegions(std::vector<frontierRegion> frontierRegions ){
    
    cout << frontierRegions.size() << endl;
    
    for (size_t i = 0; i < frontierRegions.size(); i++){
            cout << frontierRegions[i].size << " ";
            cout << frontierRegions[i].x << " ";
            cout << frontierRegions[i].y << endl;
    }

}

/**
 * This function returns the valid neighbours of a cell as a vector
 * of indices. A neighbour is considered valid if it exists within
 * the specified bounds
 *
 * @param	row						the center cell row
 * @param	col						the center cell column
 * @param	minRow					the specified lower row bounds for neighbour index
 * @param	minCol					the specified lower column bounds for neighbour index
 * @param	maxRow					the specified upper row bounds for neighbour index
 * @param	maxCol					the specified upper column bounds for neighbour index
 * @param	dr						the row index difference
 * @param	dc						the column index difference
 * @param	numberOfConnections		the connectivity
 * @return							the list of indices associated with valid neighbours
 */
vector<pair<int,int>> getNeighbours(int row, int col, int minRow, int minCol, int maxRow, int maxCol, vector<int> dr, vector<int> dc, int numberOfConnections){

    vector<pair<int,int>> output;	
    
    int neighbourRow, neighbourCol;
    
    
    for (int i = 0; i < numberOfConnections; i++){
        
        neighbourRow = row + dr[i];
        neighbourCol = col + dc[i];
        
        if (neighbourRow < minRow || neighbourRow >= maxRow) continue;
        if (neighbourCol < minCol || neighbourCol >= maxCol) continue;
        
        output.push_back(make_pair(neighbourRow,neighbourCol));
        
    }
    
    return output;

}

/**
 * This function determines if any cell from a list of indices has 
 * a specified value
 *
 * @param	grid		the input vector representing the grid contents
 * @param	width		the grid width
 * @param	cells		the list of cell indices
 * @param	value		the cell value being searched for
 * @return 				if that specified cell value is encountered	
 */
bool hasCellValue(vector<cell> grid, int width, vector<pair<int,int>> cells, int value){
    
    for (size_t i = 0; i < cells.size(); i++){
        
        if (grid[cells[i].first*width + cells[i].second] == value) return 1;
        
    }
    
    return 0;
}

/**
 * This function detects frontier edge cells and constructs a 
 * binary grid of the detected cells
 *
 * @param	occupancyGrid		the input vector representing occupancy grid contents
 * @param	width				the grid width
 * @param	height				the grid height
 * @return 						a binary grid of the map's frontier edge cells
 */
vector<cell> computeFrontierCellGrid(vector<cell> occupancyGrid, int width, int height){
    
    vector<cell> output;
    vector<pair<int,int>> neighbours;

    
    for (int i = 0; i < height; i++){
        for(int j = 0; j < width; j++){
        
            if (occupancyGrid[i*width + j] == 0){
                    
                neighbours = getNeighbours(i, j, 0, 0, height, width, dr_4, dc_4, 4);
                if (hasCellValue(occupancyGrid, width, neighbours, -1)){
                    output.push_back(1);
                } else {
                    output.push_back(0);	
                }
            } else {
                
                output.push_back(0);
                
            }
        }
    }
    
    return output;
}

/**
 * This function extracts frontier regions from a binary grid of 
 * frontier edge cells
 *
 * @param	frontierCellGrid	the input vector representing the grid of frontier edge cells
 * @param	width				the grid width
 * @param	height				the grid height
 * @return 						a list of frontier regions extracted
 */
vector<frontierRegion> computeFrontierRegions(vector<cell> frontierCellGrid, int width, int height){

    vector<frontierRegion> output;
    
    queue<pair<int, int>> cellQueue;
    vector<pair<int, int>> neighbours;
    vector<int> visited(width*height);

    // Initialize New Region
    frontierRegion newRegion;
    newRegion.size = 0;
    newRegion.x = 0;
    newRegion.y = 0;
    
    for (int i = 0; i < height; i++){
        for(int j = 0; j < width; j++){
            
            if (frontierCellGrid[i*width +j] == 1 && visited[i*width +j] == 0){
                
                // Add the Cell's Coordinate to the Queue
                cellQueue.push(make_pair(i,j));
                
                // Mark the Cell as Visited
                visited[i*width +j] = 1;
            }
            
            while (!cellQueue.empty()){
                
                // Update the New Region
                newRegion.size++;
                newRegion.x += cellQueue.front().second; 																			 // column index, j
                newRegion.y += cellQueue.front().first;  																			 // row index, i
                
                // Add the Unvisited Frontier Edge Cell Neighbours to the Queue
                neighbours.clear(); 																								 //Optional
                neighbours = getNeighbours(cellQueue.front().first, cellQueue.front().second, 0, 0, height, width, dr_8, dc_8, 8); 
                
                for(size_t k = 0; k < neighbours.size(); k++){
                    
                    size_t neighbour_i = neighbours[k].first;
                    size_t neighbour_j = neighbours[k].second;
                    
                    // Add the neighbour to the queue and mark it as visited if it has not been visited already
                    if (frontierCellGrid[neighbour_i*width +neighbour_j] == 1 && visited[neighbour_i*width +neighbour_j] == 0){
                        
                        // Add to the Queue
                        cellQueue.push(make_pair(neighbour_i,  neighbour_j));
                        
                        // Mark as Visited
                        visited[neighbour_i*width + neighbour_j] = 1;	
                    }
                }
                
                // Remove the First Element of the Queue
                cellQueue.pop();
            }
            
            if (newRegion.size != 0){
                
                // Average Summed Cell Coordinates
                newRegion.x /= newRegion.size;
                newRegion.y /= newRegion.size;
                
                // Push the New Region into the Output Vector
                output.push_back(newRegion);
                
                // Reset the New Region
                newRegion.size = 0;
                newRegion.x = 0;
                newRegion.y = 0;
                
            }

        }
    }
    
    return output;
}