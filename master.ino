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

enum IRSensorType
{
    IR_LF_LEFT,
    IR_LF_RIGHT,
    IR_INTERSECTION_LEFT,
    IR_INTERSECTION_RIGHT,
    IR_FRONT,
    NUM_IR_SENSORS
};

// Left wheel motor control pins
const int LEFT_WHEEL_MOVE_FORWARD_PIN = 5;
const int LEFT_WHEEL_MOVE_BACKWARD_PIN = 6;

// Right wheel motor control pins
const int RIGHT_WHEEL_MOVE_FORWARD_PIN = 9;
const int RIGHT_WHEEL_MOVE_BACKWARD_PIN = 10;

// Infrared sensor pins
const int IR_SENSOR_PINS[] = {11,  // IR_LF_LEFT
                              13,  // IR_LF_RIGHT
                              2,   // IR_INTERSECTION_LEFT
                              3,   // IR_INTERSECTION_RIGHT
                              12}; // IR_FRONT

int currentDirection;
int nextDirection;

struct Sensor
{
    int pin;
    bool state;
    bool lastStableState;
    unsigned long lastDebounceTime;
};

Sensor irSensors[NUM_IR_SENSORS];

const unsigned long debounceDelay = 100;

bool debounceRead(struct Sensor &sensor)
{
    bool reading = digitalRead(sensor.pin);
    if (reading != sensor.state)
    {
        sensor.lastDebounceTime = millis();
        sensor.state = reading;
    }

    if ((millis() - sensor.lastDebounceTime) > debounceDelay)
    {
        sensor.lastStableState = sensor.state;
    }

    return sensor.lastStableState;
}

struct Coordinates
{
    int row;
    int col;
};

struct Node
{
    Coordinates position;
    int g;
    int f;
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

// Heuristic function for A*
int heuristic(struct Coordinates a, struct Coordinates b)
{
    return abs(a.row - b.row) + abs(a.col - b.col);
}

Node *aStar(struct Coordinates start, struct Coordinates end)
{
    Node *start_node = new Node;
    start_node->position = start;
    start_node->g = 0;
    start_node->parent = NULL;

    // The f value of the start node is completely heuristic.
    start_node->f = heuristic(start, end);

    Node *open_list[rows * cols];
    int open_list_size = 0;
    open_list[open_list_size++] = start_node;

    bool closed_list[rows][cols] = {false};

    while (open_list_size > 0)
    {
        Node *current_node = open_list[0];
        int current_index = 0;

        // Find the node with the lowest f value
        for (int i = 1; i < open_list_size; i++)
        {
            if (open_list[i]->g + open_list[i]->f < current_node->g + current_node->f)
            {
                current_node = open_list[i];
                current_index = i;
            }
        }

        // Remove the current node from the open list
        open_list_size--;
        for (int i = current_index; i < open_list_size; i++)
        {
            open_list[i] = open_list[i + 1];
        }

        closed_list[current_node->position.row][current_node->position.col] = true;

        // Check if we have reached the goal
        if (current_node->position.row == end.row && current_node->position.col == end.col)
        {
            return current_node; // Path has been found
        }

        // Generate neighbors
        int neighbors[4][2] = {{0, 2}, {2, 0}, {0, -2}, {-2, 0}}; // Possible moves: right, down, left, up
        for (int i = 0; i < 4; i++)
        {
            Coordinates neighbor_pos = {current_node->position.row + neighbors[i][0], current_node->position.col + neighbors[i][1]};
            if (isValidMove(current_node->position, neighbor_pos) && !closed_list[neighbor_pos.row][neighbor_pos.col])
            {
                Node *neighbor_node = new Node;
                neighbor_node->position = neighbor_pos;
                neighbor_node->g = current_node->g + 1;
                neighbor_node->f = neighbor_node->g + heuristic(neighbor_pos, end);
                neighbor_node->parent = current_node;

                // Check if this node is already in the open list with a lower g (cost) value
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
                    delete neighbor_node; // Proper memory management
                }
            }
        }
    }

    return NULL; // No path found
}

// Function to find a character in the grid and return its coordinates as a Coordinates structure
Coordinates getCoordinates(char c)
{
    Coordinates coord;
    for (int i = 0; i < 11; i++)
    {
        for (int j = 0; j < 11; j++)
        {
            if (grid[i][j] == c)
            {
                coord.row = i;
                coord.col = j;
                return coord;
            }
        }
    }
    // Return a default value if character is not found
    coord.row = -1;
    coord.col = -1;
    return coord;
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

    for (int i = 0; i < NUM_IR_SENSORS; ++i)
    {
        irSensors[i].pin = IR_SENSOR_PINS[i];
        irSensors[i].state = LOW;
        irSensors[i].lastStableState = LOW;
        irSensors[i].lastDebounceTime = 0;
        pinMode(irSensors[i].pin, INPUT);
    }

    // Since the front sensor has a inverted logic, we need to set it to HIGH
    irSensors[IR_FRONT].state = HIGH;
    irSensors[IR_FRONT].lastStableState = HIGH;

    // Example usage with dynamic obstacles
    Coordinates startCoordinates = getCoordinates(START_POSITION);
    Coordinates endCoordinates = getCoordinates(END_POSITION);

    path = aStar(startCoordinates, endCoordinates);
    currentDirection = getStartDirection(START_POSITION);
    nextDirection = currentDirection;
    printPath(path);
    Serial.print("Starting facing direction: ");
    printDirection(currentDirection);
}

void controlLeftWheel(int command, int pwmValue = 220)
{
    switch (command)
    {
    case STOP:
        analogWrite(LEFT_WHEEL_MOVE_FORWARD_PIN, 0);
        analogWrite(LEFT_WHEEL_MOVE_BACKWARD_PIN, 0);
        break;
    case GO_FORWARD:
        analogWrite(LEFT_WHEEL_MOVE_FORWARD_PIN, pwmValue);
        analogWrite(LEFT_WHEEL_MOVE_BACKWARD_PIN, 0);
        break;
    case GO_BACK:
        analogWrite(LEFT_WHEEL_MOVE_FORWARD_PIN, 0);
        analogWrite(LEFT_WHEEL_MOVE_BACKWARD_PIN, pwmValue);
        break;
    }
}

void controlRightWheel(int command, int pwmValue = 220)
{
    switch (command)
    {
    case STOP:
        analogWrite(RIGHT_WHEEL_MOVE_FORWARD_PIN, 0);
        analogWrite(RIGHT_WHEEL_MOVE_BACKWARD_PIN, 0);
        break;
    case GO_FORWARD:
        analogWrite(RIGHT_WHEEL_MOVE_FORWARD_PIN, pwmValue);
        analogWrite(RIGHT_WHEEL_MOVE_BACKWARD_PIN, 0);
        break;
    case GO_BACK:
        analogWrite(RIGHT_WHEEL_MOVE_FORWARD_PIN, 0);
        analogWrite(RIGHT_WHEEL_MOVE_BACKWARD_PIN, pwmValue);
        break;
    }
}

void driveForward()
{
    int hasLeftSensorDetectedLine = debounceRead(irSensors[IR_LF_LEFT]);
    int hasRightSensorDetectedLine = debounceRead(irSensors[IR_LF_RIGHT]);

    // If both sensors detect a line, drive straight forward
    if (hasLeftSensorDetectedLine && hasRightSensorDetectedLine)
    {
        controlLeftWheel(GO_FORWARD);
        controlRightWheel(GO_FORWARD);
    }
    // If only the left sensor detects a line, turn slightly left to realign
    else if (hasLeftSensorDetectedLine && !hasRightSensorDetectedLine)
    {
        controlLeftWheel(STOP);
        controlRightWheel(GO_FORWARD);
    }
    else if (hasRightSensorDetectedLine && !hasLeftSensorDetectedLine)
    {
        controlLeftWheel(GO_FORWARD);
        controlRightWheel(STOP);
    }
}

void turnLeft()
{
    bool hasRightLineFollowerLeftFirstLine = false;
    bool hasRightLineFollowerEnteredSecondLine = false;

    while (!hasRightLineFollowerEnteredSecondLine)
    {
        bool hasRightLineFollowerDetectedLine = debounceRead(irSensors[IR_LF_RIGHT]);
        if (!hasRightLineFollowerLeftFirstLine && !hasRightLineFollowerDetectedLine)
        {
            hasRightLineFollowerLeftFirstLine = true;
        }
        if (hasRightLineFollowerLeftFirstLine && !hasRightLineFollowerEnteredSecondLine && hasRightLineFollowerDetectedLine)
        {
            hasRightLineFollowerEnteredSecondLine = true;
        }

        controlLeftWheel(GO_BACK, 200);
        controlRightWheel(GO_FORWARD, 200);
    }
}

void turnRight()
{
    bool hasLeftLineFollowerLeftFirstLine = false;
    bool hasLeftLineFollowerEnteredSecondLine = false;

    while (!hasLeftLineFollowerEnteredSecondLine)
    {
        bool hasLeftLineFollowerDetectedLine = debounceRead(irSensors[IR_LF_LEFT]);
        if (!hasLeftLineFollowerLeftFirstLine && !hasLeftLineFollowerDetectedLine)
        {
            hasLeftLineFollowerLeftFirstLine = true;
        }
        if (hasLeftLineFollowerLeftFirstLine && !hasLeftLineFollowerEnteredSecondLine && hasLeftLineFollowerDetectedLine)
        {
            hasLeftLineFollowerEnteredSecondLine = true;
        }

        controlLeftWheel(GO_FORWARD, 200);
        controlRightWheel(GO_BACK, 200);
    }
}

bool isThereObstacle()
{
    int hasFrontSensorDetectedObstacle = !debounceRead(irSensors[IR_FRONT]);
    return hasFrontSensorDetectedObstacle;
}

bool isThereIntersection()
{
    int hasLeftSensorDetectedLine = debounceRead(irSensors[IR_INTERSECTION_LEFT]);
    int hasRightSensorDetectedLine = debounceRead(irSensors[IR_INTERSECTION_RIGHT]);
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
    Serial.print("Current direction: ");
    printDirection(currentDirection);
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

void turnToNewDirection()
{
    int directionDifference = nextDirection - currentDirection;

    if (directionDifference == 1 || directionDifference == -3)
    {
        turnRight();
    }
    else if (directionDifference == -1 || directionDifference == 3)
    {
        turnLeft();
    }
    else if (directionDifference == 2 || directionDifference == -2)
    {
        turnRight();
        goBackToIntersection();
        turnRight();
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
        Node *newPath = aStar(start, end);

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
    Serial.print("Current direction: ");
    printDirection(currentDirection);
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
    int hasLeftSensorDetectedLine = debounceRead(irSensors[IR_LF_LEFT]);
    int hasRightSensorDetectedLine = debounceRead(irSensors[IR_LF_RIGHT]);

    // If both sensors detect a line, drive straight backward
    if (hasLeftSensorDetectedLine && hasRightSensorDetectedLine)
    {
        controlLeftWheel(GO_BACK);
        controlRightWheel(GO_BACK);
    }
    // If only the left sensor detects a line, turn slightly left to realign
    else if (hasLeftSensorDetectedLine && !hasRightSensorDetectedLine)
    {
        controlLeftWheel(STOP);
        controlRightWheel(GO_BACK);
    }
    // If only the right sensor detects a line, turn slightly right to realign
    else if (hasRightSensorDetectedLine && !hasLeftSensorDetectedLine)
    {
        controlLeftWheel(GO_BACK);
        controlRightWheel(STOP);
    }
}

void turnAround()
{
    bool rightSensorExitedFirstLine = false;
    bool rightSensorEnteredSecondLine = false;
    while (!rightSensorEnteredSecondLine)
    {
        bool hasRightSensorDetectedLine = debounceRead(irSensors[IR_LF_RIGHT]);
        if (!rightSensorExitedFirstLine && !hasRightSensorDetectedLine)
        {
            rightSensorExitedFirstLine = true;
        }
        if (rightSensorExitedFirstLine && !rightSensorEnteredSecondLine && hasRightSensorDetectedLine)
        {
            rightSensorEnteredSecondLine = true;
        }

        controlLeftWheel(GO_FORWARD);
        controlRightWheel(GO_BACK);
    }

    // Update the direction to be the opposite of what it was before
    currentDirection = (currentDirection + 2) % 4;
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
    for (int i = 0; i < NUM_IR_SENSORS; i++)
    {
        debounceRead(irSensors[i]);
    }

    driveForward();
    if (isThereObstacle())
    {
        Serial.println("Obstacle detected! Returning to intersection.");
        goBackToIntersection();
        Serial.println("Returned to intersection.");
        irSensors[IR_FRONT].lastStableState = !irSensors[IR_FRONT].lastStableState;
        handleObstacleAndRecalculateRoute();
        if (currentDirection != nextDirection)
        {
            turnToNewDirection();
        }
    }
    else if (isThereIntersection())
    {
        updateCurrentPosition();
        if (currentDirection != nextDirection)
        {
            turnToNewDirection();
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