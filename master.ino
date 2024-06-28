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
const int LEFT_WHEEL_MOVE_FORWARD_PIN = 5;
const int LEFT_WHEEL_MOVE_BACKWARD_PIN = 6;

// Right wheel motor control pins
const int RIGHT_WHEEL_MOVE_FORWARD_PIN = 9;
const int RIGHT_WHEEL_MOVE_BACKWARD_PIN = 10;

// Infrared sensor pins
const int IR_LF_LEFT = 11;
const int IR_LF_RIGHT = 13;
const int IR_INTERSECTION_LEFT = 2;
const int IR_INTERSECTION_RIGHT = 3;
const int IR_FRONT = 12;

int currentDirection;
int nextDirection;

unsigned long lastDebounceTime = 0; // the last time the output pin was toggled
unsigned long debounceDelay = 100; // the debounce time; increase if the output flickers

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

    TCCR0A = 0; // Clear Timer/Counter Control Registers
  TCCR0B = 0;

  // Set PWM mode to Fast PWM
  TCCR0A |= (1 << WGM00) | (1 << WGM01);

  // Set prescaler to 1 (no prescaling)
  TCCR0B |= (1 << CS00);

  // Set the output compare registers for 15 kHz frequency
  OCR0A = 106; // Ajustar duty cycle para o pino 6
  OCR0B = 106; // Ajustar duty cycle para o pino 5

  // Enable PWM nos pinos 5 e 6
  TCCR0A |= (1 << COM0A1) | (1 << COM0B1);

  // Configurar Timer1 para 15 kHz nos pinos 9 e 10
  TCCR1A = 0; // Clear Timer/Counter Control Registers
  TCCR1B = 0;

  // Set PWM mode to Fast PWM 8-bit
  TCCR1A |= (1 << WGM10);
  TCCR1B |= (1 << WGM12);

  // Set prescaler to 1 (no prescaling)
  TCCR1B |= (1 << CS10);

  // Set the output compare registers for 15 kHz frequency
  OCR1A = 106; // Ajustar duty cycle para o pino 9
  OCR1B = 106; // Ajustar duty cycle para o pino 10

  // Enable PWM nos pinos 9 e 10
  TCCR1A |= (1 << COM1A1) | (1 << COM1B1);

    pinMode(IR_LF_LEFT, INPUT);
    pinMode(IR_LF_RIGHT, INPUT);

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
    int hasLeftSensorDetectedLine = digitalRead(IR_LF_LEFT);
    int hasRightSensorDetectedLine = digitalRead(IR_LF_RIGHT);

    // If only left line detects a line, turn slightly right to realign
    // if (hasLeftSensorDetectedLine && !hasRightSensorDetectedLine)
    // {
    //     controlLeftWheel(STOP);
    //     controlRightWheel(GO_FORWARD);
    // }
    // // If only right line detects a line, turn slightly left to realign
    // else if (hasRightSensorDetectedLine && !hasLeftSensorDetectedLine)
    // {
    //     controlLeftWheel(GO_FORWARD);
    //     controlRightWheel(STOP);
    // }
    // else
    // {
    //     controlLeftWheel(GO_FORWARD);
    //     controlRightWheel(GO_FORWARD);
    // }
    //If both sensors detect a line, drive straight forward
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
     else
     {
         controlLeftWheel(STOP);
         controlRightWheel(STOP);
     }
}

void turnLeft()
{
    bool rightSensorExitedFirstLine = false;
    bool rightSensorEnteredSecondLine = false;
    bool rightSensorExitedSecondLine = false;
   // int pwmValue = 255; // Start with max PWM value

    while (!rightSensorExitedSecondLine && !digitalRead(IR_LF_RIGHT))
    {
        // Read from the right sensor
        int hasRightSensorDetectedLine = digitalRead(IR_INTERSECTION_RIGHT);

        // Check if the right sensor has exited the first line
        if (!rightSensorExitedFirstLine && !hasRightSensorDetectedLine)
        {
            rightSensorExitedFirstLine = true;
            //pwmValue -= pwmValue * 0.10; // Reduce PWM value by 5%
        }

        // Check if the right sensor has entered the second line
        if (rightSensorExitedFirstLine && !rightSensorEnteredSecondLine && hasRightSensorDetectedLine)
        {
            rightSensorEnteredSecondLine = true;
            //pwmValue -= pwmValue * 0.10; // Reduce PWM value by 5%
        }

        // Check if the right sensor has exited the second line
        if (rightSensorEnteredSecondLine && !rightSensorExitedSecondLine && !hasRightSensorDetectedLine)
        {
            rightSensorExitedSecondLine = true;
            //pwmValue -= pwmValue * 0.10; // Reduce PWM value by 5%
        }

        // Turn left
        controlLeftWheel(GO_BACK,190);
        controlRightWheel(GO_FORWARD, 190);
    }
}

void turnRight()
{
    bool leftSensorExitedFirstLine = false;
    bool leftSensorEnteredSecondLine = false;
    bool leftSensorExitedSecondLine = false;
    //int pwmValue = 255; // Start with max PWM value

    while (!leftSensorExitedSecondLine && !digitalRead(IR_LF_LEFT))
    {
        // Read from the left sensor
        int hasLeftSensorDetectedLine = digitalRead(IR_INTERSECTION_LEFT);

        // Check if the left sensor has exited the first line
        if (!leftSensorExitedFirstLine && !hasLeftSensorDetectedLine)
        {
            leftSensorExitedFirstLine = true;
            //pwmValue -= pwmValue * 0.10; // Reduce PWM value by 5%
        }

        // Check if the left sensor has entered the second line
        if (leftSensorExitedFirstLine && !leftSensorEnteredSecondLine && hasLeftSensorDetectedLine)
        {
            leftSensorEnteredSecondLine = true;
            //pwmValue -= pwmValue * 0.10; // Reduce PWM value by 5%
        }

        // Check if the left sensor has exited the second line
        if (leftSensorEnteredSecondLine && !leftSensorExitedSecondLine && !hasLeftSensorDetectedLine)
        {
            leftSensorExitedSecondLine = true;
            //pwmValue -= pwmValue * 0.10; // Reduce PWM value by 5%
        }

        // Turn right
        controlLeftWheel(GO_FORWARD, 190);
        controlRightWheel(GO_BACK,190);
    }
}

bool isThereObstacle()
{
    int hasFrontSensorDetectedObstacle = !digitalRead(IR_FRONT);
    return hasFrontSensorDetectedObstacle;
}

bool isThereIntersection()
{
    int hasLeftSensorDetectedLine = digitalRead(IR_INTERSECTION_LEFT);
    int hasRightSensorDetectedLine = digitalRead(IR_INTERSECTION_RIGHT);
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
    int hasLeftSensorDetectedLine = digitalRead(IR_LF_LEFT);
    int hasRightSensorDetectedLine = digitalRead(IR_LF_RIGHT);

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

bool debounceRead(int pin)
{
    int reading = digitalRead(pin);
    static int lastReading = LOW;
    static bool debouncedReading = LOW;

    if (reading != lastReading) {
        lastDebounceTime = millis();
    }

    if ((millis() - lastDebounceTime) > debounceDelay) {
        if (reading != debouncedReading) {
            debouncedReading = reading;
        }
    }

    lastReading = reading;
    return debouncedReading;
}

void turnAround()
{
    // Assume we're turning right
    bool rightSensorExitedFirstLine = false;
    bool rightSensorEnteredSecondLine = false;
    while(!rightSensorEnteredSecondLine){
        bool hasRightSensorDetectedLine = debounceRead(IR_LF_RIGHT);
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

    // Drive until the intersection
    while (!isThereIntersection())
    {
        driveForward();
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
        Serial.println("Obstacle detected! Returning to intersection.");
        goBackToIntersection();
        Serial.println("Returned to intersection.");
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