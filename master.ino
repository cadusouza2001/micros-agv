/**
 * AGV (Automated Guided Vehicle) control system
 * Universidade do Vale do Rio dos Sinos - UNISINOS
 * Computer Engineering - Microprocessor Systems
 *
 * Authors:
 * - Carlos Eduardo
 * - Klaus Becker
 */
#define START_POSITION 'H'
#define END_POSITION '3'
// Define the possible facing directions
enum
{
    NORTH = 0,
    EAST,
    SOUTH,
    WEST
};

enum
{
    STOP = 0,
    GO_FORWARD,
    GO_BACK
};

// Left wheel motor control pins
const int LEFT_WHEEL_MOVE_FORWARD_PIN = 6;
const int LEFT_WHEEL_MOVE_BACKWARD_PIN = 7;

// Right wheel motor control pins
const int RIGHT_WHEEL_MOVE_FORWARD_PIN = 4;
const int RIGHT_WHEEL_MOVE_BACKWARD_PIN = 5;

// Infrared sensor pins
const int IR_LEFT = 10;
const int IR_RIGHT = 8;
const int IR_FRONT = 9;

int currentDirection;
int nextDirection;

struct Coordinates
{
    int row;
    int col;
};

struct Node
{
    Coordinates position;
    int g;
    Node *parent;
};

Node *path;

// Define the grid dimensions
const int rows = 11;
const int cols = 11;

// Define the grid
char grid[rows][cols] = {
    // 0    1    2    3    4    5    6    7    8    9   10
    {' ', ' ', 'E', ' ', 'F', ' ', 'G', ' ', 'H', ' ', ' '},  // 0
    {' ', ' ', '|', ' ', '|', ' ', '|', ' ', '|', ' ', ' '},  // 1
    {'1', '-', '+', '-', '+', '-', '+', '-', '+', '-', '5'},  // 2
    {' ', ' ', '|', ' ', '|', ' ', '|', ' ', '|', ' ', ' '},  // 3
    {'2', '-', '+', '-', '+', '-', '+', '-', '+', '-', '6'},  // 4
    {' ', ' ', '|', ' ', '|', ' ', '|', ' ', '|', ' ', ' '},  // 5
    {'3', '-', '+', '-', '+', '-', '+', '-', '+', '-', '7'},  // 6
    {' ', ' ', '|', ' ', '|', ' ', '|', ' ', '|', ' ', ' '},  // 7
    {'4', '-', '+', '-', '+', '-', '+', '-', '+', '-', '8'},  // 8
    {' ', ' ', '|', ' ', '|', ' ', '|', ' ', '|', ' ', ' '},  // 9
    {' ', ' ', 'A', ' ', 'B', ' ', 'C', ' ', 'D', ' ', ' '}}; // 10

// Function to find a character in the grid and return its coordinates
void findCharInGrid(char c, int &x, int &y)
{
    for (int i = 0; i < 11; i++)
    {
        for (int j = 0; j < 11; j++)
        {
            if (grid[i][j] == c)
            {
                x = i;
                y = j;
                return;
            }
        }
    }
}

bool isValidMove(struct Coordinates current_pos, struct Coordinates next_pos)
{
    int row = current_pos.row;
    int col = current_pos.col;
    int next_row = next_pos.row;
    int next_col = next_pos.col;

    if (next_row < 0 || next_row >= rows || next_col < 0 || next_col >= cols)
    {
        return false;
    }

    if (grid[next_row][next_col] == ' ' || grid[next_row][next_col] == 'x')
    { // Consider 'x' as obstacle
        return false;
    }

    if (row == next_row)
    { // Horizontal move
        if (next_col - col == 2 && grid[row][col + 1] != '-')
        {
            return false;
        }
        else if (next_col - col == -2 && grid[row][col - 1] != '-')
        {
            return false;
        }
    }
    else if (col == next_col)
    { // Vertical move
        if (next_row - row == 2 && grid[row + 1][col] != '|')
        {
            return false;
        }
        else if (next_row - row == -2 && grid[row - 1][col] != '|')
        {
            return false;
        }
    }

    return true;
}

Node *dijkstra(struct Coordinates start, struct Coordinates end)
{
    Node *start_node = new Node;
    start_node->position.row = start.row;
    start_node->position.col = start.col;
    start_node->g = 0;
    start_node->parent = NULL;

    Node *open_list[rows * cols];
    int open_list_size = 0;
    open_list[open_list_size++] = start_node;

    bool closed_list[rows][cols] = {false};

    while (open_list_size > 0)
    {
        Node *current_node = open_list[0];
        int current_index = 0;

        for (int i = 1; i < open_list_size; i++)
        {
            if (open_list[i]->g < current_node->g)
            {
                current_node = open_list[i];
                current_index = i;
            }
        }

        open_list_size--;
        for (int i = current_index; i < open_list_size; i++)
        {
            open_list[i] = open_list[i + 1];
        }

        closed_list[current_node->position.row][current_node->position.col] = true;

        if (current_node->position.row == end.row && current_node->position.col == end.col)
        {
            return current_node;
        }

        int neighbors[4][2] = {
            {0, 2}, {2, 0}, {0, -2}, {-2, 0} // Possible moves: right, down, left, up
        };

        for (int i = 0; i < 4; i++)
        {
            Coordinates neighbor_pos = {current_node->position.row + neighbors[i][0], current_node->position.col + neighbors[i][1]};

            if (isValidMove(Coordinates{current_node->position.row, current_node->position.col}, neighbor_pos) && !closed_list[neighbor_pos.row][neighbor_pos.col])
            {
                Node *neighbor_node = new Node;
                neighbor_node->position.row = neighbor_pos.row;
                neighbor_node->position.col = neighbor_pos.col;
                neighbor_node->g = current_node->g + 1;
                neighbor_node->parent = current_node;

                bool skip = false;
                for (int j = 0; j < open_list_size; j++)
                {
                    if (open_list[j]->position.row == neighbor_node->position.row && open_list[j]->position.col == neighbor_node->position.col && open_list[j]->g <= neighbor_node->g)
                    {
                        skip = true;
                        break;
                    }
                }

                if (!skip)
                {
                    open_list[open_list_size++] = neighbor_node;
                }
                else
                {
                    delete neighbor_node;
                }
            }
        }
    }
    return NULL;
}

// Function to parse the input and find the start and end coordinates
void parseInput(char startChar, char endChar, int &startX, int &startY, int &endX, int &endY)
{
    findCharInGrid(startChar, startX, startY);
    findCharInGrid(endChar, endX, endY);
}

int getStartDirection(char startChar)
{
    if (startChar >= 'E' && startChar <= 'H')
    {
        return SOUTH;
    }
    else if (startChar >= '1' && startChar <= '4')
    {
        return EAST;
    }
    else if (startChar >= '5' && startChar <= '8')
    {
        return WEST;
    }
    else if (startChar >= 'A' && startChar <= 'D')
    {
        return NORTH;
    }
}

void printPath(struct Node *path)
{
    for (int i = 0; i < rows; i++)
    {
        for (int j = 0; j < cols; j++)
        {
            bool isPath = false;
            for (Node *node = path; node != NULL; node = node->parent)
            {
                if (node->position.row == i && node->position.col == j)
                {
                    Serial.print('P'); // Mark this cell as part of the path
                    isPath = true;
                    break;
                }
            }
            if (!isPath)
            {
                Serial.print(grid[i][j]); // Print the original grid content
            }
        }
        Serial.println();
    }
}

void setup()
{
    Serial.begin(9600);
    // Motor control pin modes
    pinMode(LEFT_WHEEL_MOVE_FORWARD_PIN, OUTPUT);
    pinMode(LEFT_WHEEL_MOVE_BACKWARD_PIN, OUTPUT);
    pinMode(RIGHT_WHEEL_MOVE_FORWARD_PIN, OUTPUT);
    pinMode(RIGHT_WHEEL_MOVE_BACKWARD_PIN, OUTPUT);

    // Example usage with dynamic obstacles
    int startX, startY, endX, endY;
    parseInput(START_POSITION, END_POSITION, startX, startY, endX, endY);

    path = dijkstra(Coordinates{startX, startY}, Coordinates{endX, endY});
    currentDirection = getStartDirection(START_POSITION);
    nextDirection = currentDirection;
    printPath(path);
    Serial.print("Starting facing direction: ");
    switch (currentDirection)
    {
    case NORTH:
        Serial.println("NORTH");
        break;
    case EAST:
        Serial.println("EAST");
        break;
    case SOUTH:
        Serial.println("SOUTH");
        break;
    case WEST:
        Serial.println("WEST");
        break;
    }
}

void controlLeftWheel(int command)
{
    switch (command)
    {
    case STOP:
        digitalWrite(LEFT_WHEEL_MOVE_FORWARD_PIN, LOW);
        digitalWrite(LEFT_WHEEL_MOVE_BACKWARD_PIN, LOW);
        break;
    case GO_FORWARD:
        digitalWrite(LEFT_WHEEL_MOVE_FORWARD_PIN, HIGH);
        digitalWrite(LEFT_WHEEL_MOVE_BACKWARD_PIN, LOW);
        break;
    case GO_BACK:
        digitalWrite(LEFT_WHEEL_MOVE_FORWARD_PIN, LOW);
        digitalWrite(LEFT_WHEEL_MOVE_BACKWARD_PIN, HIGH);
        break;
    }
}

void controlRightWheel(int command)
{
    switch (command)
    {
    case STOP:
        digitalWrite(RIGHT_WHEEL_MOVE_FORWARD_PIN, LOW);
        digitalWrite(RIGHT_WHEEL_MOVE_BACKWARD_PIN, LOW);
        break;
    case GO_FORWARD:
        digitalWrite(RIGHT_WHEEL_MOVE_FORWARD_PIN, HIGH);
        digitalWrite(RIGHT_WHEEL_MOVE_BACKWARD_PIN, LOW);
        break;
    case GO_BACK:
        digitalWrite(RIGHT_WHEEL_MOVE_FORWARD_PIN, LOW);
        digitalWrite(RIGHT_WHEEL_MOVE_BACKWARD_PIN, HIGH);
        break;
    }
}

void driveForward()
{
    int hasLeftSensorDetectedLine = digitalRead(IR_LEFT);
    int hasRightSensorDetectedLine = digitalRead(IR_RIGHT);

    // If only the left sensor detects a line, turn slightly right to realign
    if (hasLeftSensorDetectedLine && !hasRightSensorDetectedLine)
    {
        controlLeftWheel(STOP);
        controlRightWheel(GO_FORWARD);
    }
    // If only the right sensor detects a line, turn slightly left to realign
    else if (hasRightSensorDetectedLine && !hasLeftSensorDetectedLine)
    {
        controlLeftWheel(GO_FORWARD);
        controlRightWheel(STOP);
    }
    // If neither sensor detects a line, drive straight forward
    else
    {
        controlLeftWheel(GO_FORWARD);
        controlRightWheel(GO_FORWARD);
    }
}

void turnLeft(bool isAtIntersection)
{
    bool rightSensorEnteredFirstLine = false;
    bool rightSensorExitedFirstLine = false;
    bool rightSensorEnteredSecondLine = false;
    bool rightSensorExitedSecondLine = false;

    while (!rightSensorExitedSecondLine)
    {
        // Read from the right sensor
        int hasRightSensorDetectedLine = digitalRead(IR_RIGHT);

        // If not at an intersection, check if the left sensor has entered the first line
        if (isAtIntersection || !rightSensorEnteredFirstLine && hasRightSensorDetectedLine)
        {
            rightSensorEnteredFirstLine = true;
        }

        // Check if the right sensor has exited the first line
        if (rightSensorEnteredFirstLine && !rightSensorExitedFirstLine && !hasRightSensorDetectedLine)
        {
            rightSensorExitedFirstLine = true;
        }

        // Check if the right sensor has entered the second line
        if (rightSensorExitedFirstLine && !rightSensorEnteredSecondLine && hasRightSensorDetectedLine)
        {
            rightSensorEnteredSecondLine = true;
        }

        // Check if the right sensor has exited the second line
        if (rightSensorEnteredSecondLine && !rightSensorExitedSecondLine && !hasRightSensorDetectedLine)
        {
            rightSensorExitedSecondLine = true;
        }

        // Turn left
        controlLeftWheel(STOP);
        controlRightWheel(GO_FORWARD);
    }
}

void turnRight(bool isAtIntersection)
{
    bool leftSensorEnteredFirstLine = false;
    bool leftSensorExitedFirstLine = false;
    bool leftSensorEnteredSecondLine = false;
    bool leftSensorExitedSecondLine = false;

    while (!leftSensorExitedSecondLine)
    {
        // Read from the left sensor
        int hasLeftSensorDetectedLine = digitalRead(IR_LEFT);

        // If not at an intersection, check if the left sensor has entered the first line
        if (isAtIntersection || !leftSensorEnteredFirstLine && hasLeftSensorDetectedLine)
        {
            leftSensorEnteredFirstLine = true;
        }

        // Check if the left sensor has exited the first line
        if (leftSensorEnteredFirstLine && !leftSensorExitedFirstLine && !hasLeftSensorDetectedLine)
        {
            leftSensorExitedFirstLine = true;
        }

        // Check if the left sensor has entered the second line
        if (leftSensorExitedFirstLine && !leftSensorEnteredSecondLine && hasLeftSensorDetectedLine)
        {
            leftSensorEnteredSecondLine = true;
        }

        // Check if the left sensor has exited the second line
        if (leftSensorEnteredSecondLine && !leftSensorExitedSecondLine && !hasLeftSensorDetectedLine)
        {
            leftSensorExitedSecondLine = true;
        }

        // Turn right
        controlLeftWheel(GO_FORWARD);
        controlRightWheel(STOP);
    }
}

bool isThereObstacle()
{
    int hasFrontSensorDetectedObstacle = !digitalRead(IR_FRONT);
    return hasFrontSensorDetectedObstacle;
}

bool isThereIntersection()
{
    int hasLeftSensorDetectedLine = digitalRead(IR_LEFT);
    int hasRightSensorDetectedLine = digitalRead(IR_RIGHT);
    return hasLeftSensorDetectedLine && hasRightSensorDetectedLine;
}

void updateCurrentPosition()
{
    if (path == NULL || path->parent == NULL)
    {
        // If path is empty or only has one node, we cannot update the position.
        return;
    }

    // Find the second to last node and the last node
    Node *thirdToLast = path;
    Node *secondToLast = path->parent;
    Node *last = path->parent->parent;
    while (last->parent != NULL)
    {
        thirdToLast = secondToLast;
        secondToLast = last;
        last = last->parent;
    }

    if (last != NULL)
    {
        delete last;
        secondToLast->parent = NULL;
    }
    else
    {
        delete secondToLast;
        thirdToLast->parent = NULL;
    }
    // Now, secondToLast is the new last node in the path
    // Determine the next direction based on the positions of the second to last node and the last node
    nextDirection = getNextDirection(thirdToLast, secondToLast);
    // Remove the last node
    printPath(path);
    Serial.println("Updated current position: ");
    Serial.print(secondToLast->position.row);
    Serial.print(", ");
    Serial.println(secondToLast->position.col);
    Serial.print("Next direction: ");
    printDirection(nextDirection);
}

int getNextDirection(Node *nextPosition, Node *currentPosition)
{
    int deltaX = nextPosition->position.col - currentPosition->position.col;
    int deltaY = nextPosition->position.row - currentPosition->position.row;

    if (deltaX == 2)
    {
        return EAST;
    }
    else if (deltaX == -2)
    {
        return WEST;
    }
    else if (deltaY == 2)
    {
        return SOUTH;
    }
    else if (deltaY == -2)
    {
        return NORTH;
    }
}

void turnToNewDirection(bool isAtIntersection)
{
    int directionDifference = nextDirection - currentDirection;

    if (directionDifference == 1 || directionDifference == -3)
    {
        turnRight(isAtIntersection);
    }
    else if (directionDifference == -1 || directionDifference == 3)
    {
        turnLeft(isAtIntersection);
    }

    currentDirection = nextDirection;
    Serial.print("Turned to new direction: ");
    printDirection(currentDirection);
}

void printDirection(int direction)
{
    switch (direction)
    {
    case NORTH:
        Serial.println("NORTH");
        break;
    case EAST:
        Serial.println("EAST");
        break;
    case SOUTH:
        Serial.println("SOUTH");
        break;
    case WEST:
        Serial.println("WEST");
        break;
    }
}

void handleObstacleAndRecalculateRoute()
{
    if (path == NULL || path->parent == NULL)
    {
        // Path is too short, cannot handle obstacle
        return;
    }

    Node *secondToLast = path;
    Node *last = path->parent;
    while (last->parent != NULL)
    {
        secondToLast = last;
        last = last->parent;
    }

    // Calculate the position between last and secondToLast nodes
    int obstacleRow = (last->position.row + secondToLast->position.row) / 2;
    int obstacleColumn = (last->position.col + secondToLast->position.col) / 2;

    // Check if the position is valid for an obstacle ('-' or '|')
    if (grid[obstacleRow][obstacleColumn] == '-' || grid[obstacleRow][obstacleColumn] == '|')
    {
        // Mark the position as an obstacle
        grid[obstacleRow][obstacleColumn] = 'x';

        // Recalculate the route from the current position to the destination
        Coordinates start = {last->position.row, last->position.col};
        Coordinates end = {path->position.row, path->position.col};
        // You might need to implement a way to retrieve or store the end position
        Node *newPath = dijkstra(start, end);

        // Update the path with the new route
        // Make sure to properly delete/free the old path to avoid memory leaks
        deletePath(path);
        path = newPath;
        Serial.println("Recalculated path:");
        printPath(path);

        // Optionally, print the new path or take other actions as needed
    }
    secondToLast = path;
    last = path->parent;

    while (last->parent != NULL)
    {
        secondToLast = last;
        last = last->parent;
    }

    nextDirection = getNextDirection(secondToLast, last);
    Serial.print("Next direction: ");
    printDirection(nextDirection);
}

void deletePath(Node *node)
{
    if (node == NULL)
    {
        return;
    }

    deletePath(node->parent);
    delete node;
}

void driveBackward()
{
    int hasLeftSensorDetectedLine = digitalRead(IR_LEFT);
    int hasRightSensorDetectedLine = digitalRead(IR_RIGHT);

    // If only the left sensor detects a line, turn slightly right to realign
    if (hasLeftSensorDetectedLine && !hasRightSensorDetectedLine)
    {
        controlLeftWheel(STOP);
        controlRightWheel(GO_BACK);
    }
    // If only the right sensor detects a line, turn slightly left to realign
    else if (hasRightSensorDetectedLine && !hasLeftSensorDetectedLine)
    {
        controlLeftWheel(GO_BACK);
        controlRightWheel(STOP);
    }
    // If neither sensor detects a line, drive straight forward
    else
    {
        controlLeftWheel(GO_BACK);
        controlRightWheel(GO_BACK);
    }
}

void goBackToIntersection()
{
    while (!isThereIntersection())
    {
        driveBackward();
    }
}

void loop()
{

    driveForward();
    if (isThereObstacle())
    {
        Serial.println("Obstacle detected!");
        goBackToIntersection();
        handleObstacleAndRecalculateRoute();
        if (currentDirection != nextDirection)
        {
            turnToNewDirection(false);
        }
    }
    else if (isThereIntersection())
    {
        updateCurrentPosition();
        if (currentDirection != nextDirection)
        {
            turnToNewDirection(true);
        }
        else
        {
            while (isThereIntersection())
            {
                driveForward();
            }
        }
    }
}
