#include "stdio.h"
#include "stdlib.h"
#include "SDL2/SDL.h"
#include "SDL2_gfx-1.0.1/SDL2_gfxPrimitives.h"
#include "time.h"

#include "formulas.h"
#include "wall.h"
#include "robot.h"

int done = 0;

// ------------------------------------------------------- Formulas ------------------------------------------------------

int checkOverlap(int object1X, int object1width,
                 int object1Y, int object1height,
                 int object2X, int object2width,
                 int object2Y, int object2height)
{
    int xOverlap, yOverlap;
    int min1st, max1st, min2nd, max2nd;

    min1st = object1X;
    max1st = object1X + object1width;
    min2nd = object2X;
    max2nd = object2X + object2width;
    xOverlap = ((min2nd <= max1st) && (max1st <= max2nd)) || ((min1st <= min2nd) && (min2nd <= max1st)) || ((min2nd <= min1st) && (min1st <= max2nd)) || ((min1st <= max2nd) && (max2nd <= max1st));

    min1st = object1Y;
    max1st = object1Y + object1height;
    min2nd = object2Y;
    max2nd = object2Y + object2height;
    yOverlap = ((min2nd <= max1st) && (max1st <= max2nd)) || ((min1st <= min2nd) && (min2nd <= max1st)) || ((min2nd <= min1st) && (min1st <= max2nd)) || ((min1st <= max2nd) && (max2nd <= max1st));

    return (xOverlap && yOverlap);
}

// ------------------------------------------------------- Formulas ------------------------------------------------------

// ------------------------------------------------------- Robot ------------------------------------------------------

void setup_robot(struct Robot *robot)
{
    robot->x = OVERALL_WINDOW_WIDTH / 2 - 50;
    robot->y = OVERALL_WINDOW_HEIGHT - 50;
    robot->true_x = OVERALL_WINDOW_WIDTH / 2 - 50;
    robot->true_y = OVERALL_WINDOW_HEIGHT - 50;
    robot->width = ROBOT_WIDTH;
    robot->height = ROBOT_HEIGHT;
    robot->direction = 0;
    robot->angle = 0;
    robot->currentSpeed = 0;
    robot->crashed = 0;
    robot->auto_mode = 0;

    printf("Press arrow keys to move manually, or enter to move automatically\n\n");
}

int robot_off_screen(struct Robot *robot)
{
    if (robot->x < 0 || robot->y < 0)
    {
        return 0;
    }
    if (robot->x > OVERALL_WINDOW_WIDTH || robot->y > OVERALL_WINDOW_HEIGHT)
    {
        return 0;
    }
    return 1;
}

int checkRobotHitWall(struct Robot *robot, struct Wall *wall)
{

    int overlap = checkOverlap(robot->x, robot->width, robot->y, robot->height,
                               wall->x, wall->width, wall->y, wall->height);

    return overlap;
}

int checkRobotHitWalls(struct Robot *robot, struct Wall_collection *head)
{
    struct Wall_collection *ptr = head;
    int hit = 0;

    while (ptr != NULL)
    {
        hit = (hit || checkRobotHitWall(robot, &ptr->wall));
        ptr = ptr->next;
    }
    return hit;
}

int checkRobotReachedEnd(struct Robot *robot, int x, int y, int width, int height)
{

    int overlap = checkOverlap(robot->x, robot->width, robot->y, robot->height,
                               x, width, y, height);

    return overlap;
}

void robotCrash(struct Robot *robot)
{
    robot->currentSpeed = 0;
    if (!robot->crashed)
        printf("Ouchies!!!!!\n\nPress space to start again\n");
    robot->crashed = 1;
}

void robotSuccess(struct Robot *robot, int msec)
{
    robot->currentSpeed = 0;
    if (!robot->crashed)
    {
        printf("Success!!!!!\n\n");
        printf("Time taken %d seconds %d milliseconds \n", msec / 1000, msec % 1000);
        printf("Press space to start again\n");
    }
    robot->crashed = 1;
}

int checkRobotSensor(int x, int y, int sensorSensitivityLength, struct Wall *wall)
{
    //viewing_region of sensor is a square of 2 pixels * chosen length of sensitivity
    int overlap = checkOverlap(x, 2, y, sensorSensitivityLength,
                               wall->x, wall->width, wall->y, wall->height);

    return overlap;
}

int checkRobotSensorFrontRightAllWalls(struct Robot *robot, struct Wall_collection *head)
{
    struct Wall_collection *ptr, *head_store;
    int i;
    double xDir, yDir;
    int robotCentreX, robotCentreY, xTL, yTL;
    int score, hit;

    int sensorSensitivityLength = floor(SENSOR_VISION / 5);

    head_store = head;
    robotCentreX = robot->x + ROBOT_WIDTH / 2;
    robotCentreY = robot->y + ROBOT_HEIGHT / 2;
    score = 0;

    for (i = 0; i < 5; i++)
    {
        ptr = head_store;
        xDir = round(robotCentreX + (ROBOT_WIDTH / 2 - 2) * cos((robot->angle) * PI / 180) - (-ROBOT_HEIGHT / 2 - SENSOR_VISION + sensorSensitivityLength * i) * sin((robot->angle) * PI / 180));
        yDir = round(robotCentreY + (ROBOT_WIDTH / 2 - 2) * sin((robot->angle) * PI / 180) + (-ROBOT_HEIGHT / 2 - SENSOR_VISION + sensorSensitivityLength * i) * cos((robot->angle) * PI / 180));
        xTL = (int)xDir;
        yTL = (int)yDir;
        hit = 0;

        while (ptr != NULL)
        {
            hit = (hit || checkRobotSensor(xTL, yTL, sensorSensitivityLength, &ptr->wall));
            ptr = ptr->next;
        }
        if (hit)
            score = i;
    }
    return score;
}

int checkRobotSensorFrontLeftAllWalls(struct Robot *robot, struct Wall_collection *head)
{
    struct Wall_collection *ptr, *head_store;
    int i;
    double xDir, yDir;
    int robotCentreX, robotCentreY, xTL, yTL;
    int score, hit;
    int sensorSensitivityLength;

    head_store = head;
    robotCentreX = robot->x + ROBOT_WIDTH / 2;
    robotCentreY = robot->y + ROBOT_HEIGHT / 2;
    score = 0;
    sensorSensitivityLength = floor(SENSOR_VISION / 5);

    for (i = 0; i < 5; i++)
    {
        ptr = head_store;
        xDir = round(robotCentreX + (-ROBOT_WIDTH / 2) * cos((robot->angle) * PI / 180) - (-ROBOT_HEIGHT / 2 - SENSOR_VISION + sensorSensitivityLength * i) * sin((robot->angle) * PI / 180));
        yDir = round(robotCentreY + (-ROBOT_WIDTH / 2) * sin((robot->angle) * PI / 180) + (-ROBOT_HEIGHT / 2 - SENSOR_VISION + sensorSensitivityLength * i) * cos((robot->angle) * PI / 180));
        xTL = (int)xDir;
        yTL = (int)yDir;
        hit = 0;

        while (ptr != NULL)
        {
            hit = (hit || checkRobotSensor(xTL, yTL, sensorSensitivityLength, &ptr->wall));
            ptr = ptr->next;
        }
        if (hit)
            score = i;
    }
    return score;
}

void robotUpdate(struct SDL_Renderer *renderer, struct Robot *robot)
{
    double xDir, yDir;

    int robotCentreX, robotCentreY, xTR, yTR, xTL, yTL, xBR, yBR, xBL, yBL;
    SDL_SetRenderDrawColor(renderer, 100, 100, 100, 255);

    /*
    //Other Display options:
    // The actual square which the robot is tested against (not so nice visually with turns, but easier
    // to test overlap
    SDL_Rect rect = {robot->x, robot->y, robot->height, robot->width};
    SDL_SetRenderDrawColor(renderer, 80, 80, 80, 255);
    SDL_RenderDrawRect(renderer, &rect);
    SDL_RenderFillRect(renderer, &rect);
    */
    /*
    //Center Line of Robot. Line shows the direction robot is facing
    xDir = -30 * sin(-robot->angle*PI/180);
    yDir = -30 * cos(-robot->angle*PI/180);
    xDirInt = robot->x+ROBOT_WIDTH/2+ (int) xDir;
    yDirInt = robot->y+ROBOT_HEIGHT/2+ (int) yDir;
    SDL_RenderDrawLine(renderer,robot->x+ROBOT_WIDTH/2, robot->y+ROBOT_HEIGHT/2, xDirInt, yDirInt);
    */

    //Rotating Square
    //Vector rotation to work out corners x2 = x1cos(angle)-y1sin(angle), y2 = x1sin(angle)+y1cos(angle)
    robotCentreX = robot->x + ROBOT_WIDTH / 2;
    robotCentreY = robot->y + ROBOT_HEIGHT / 2;

    xDir = round(robotCentreX + (ROBOT_WIDTH / 2) * cos((robot->angle) * PI / 180) - (-ROBOT_HEIGHT / 2) * sin((robot->angle) * PI / 180));
    yDir = round(robotCentreY + (ROBOT_WIDTH / 2) * sin((robot->angle) * PI / 180) + (-ROBOT_HEIGHT / 2) * cos((robot->angle) * PI / 180));
    xTR = (int)xDir;
    yTR = (int)yDir;

    xDir = round(robotCentreX + (ROBOT_WIDTH / 2) * cos((robot->angle) * PI / 180) - (ROBOT_HEIGHT / 2) * sin((robot->angle) * PI / 180));
    yDir = round(robotCentreY + (ROBOT_WIDTH / 2) * sin((robot->angle) * PI / 180) + (ROBOT_HEIGHT / 2) * cos((robot->angle) * PI / 180));
    xBR = (int)xDir;
    yBR = (int)yDir;

    xDir = round(robotCentreX + (-ROBOT_WIDTH / 2) * cos((robot->angle) * PI / 180) - (ROBOT_HEIGHT / 2) * sin((robot->angle) * PI / 180));
    yDir = round(robotCentreY + (-ROBOT_WIDTH / 2) * sin((robot->angle) * PI / 180) + (ROBOT_HEIGHT / 2) * cos((robot->angle) * PI / 180));
    xBL = (int)xDir;
    yBL = (int)yDir;

    xDir = round(robotCentreX + (-ROBOT_WIDTH / 2) * cos((robot->angle) * PI / 180) - (-ROBOT_HEIGHT / 2) * sin((robot->angle) * PI / 180));
    yDir = round(robotCentreY + (-ROBOT_WIDTH / 2) * sin((robot->angle) * PI / 180) + (-ROBOT_HEIGHT / 2) * cos((robot->angle) * PI / 180));
    xTL = (int)xDir;
    yTL = (int)yDir;

    SDL_RenderDrawLine(renderer, xTR, yTR, xBR, yBR);
    SDL_RenderDrawLine(renderer, xBR, yBR, xBL, yBL);
    SDL_RenderDrawLine(renderer, xBL, yBL, xTL, yTL);
    SDL_RenderDrawLine(renderer, xTL, yTL, xTR, yTR);

    //Front Right Sensor
    int sensor_sensitivity = floor(SENSOR_VISION / 5);
    int i;
    for (i = 0; i < 5; i++)
    {
        xDir = round(robotCentreX + (ROBOT_WIDTH / 2 - 2) * cos((robot->angle) * PI / 180) - (-ROBOT_HEIGHT / 2 - SENSOR_VISION + sensor_sensitivity * i) * sin((robot->angle) * PI / 180));
        yDir = round(robotCentreY + (ROBOT_WIDTH / 2 - 2) * sin((robot->angle) * PI / 180) + (-ROBOT_HEIGHT / 2 - SENSOR_VISION + sensor_sensitivity * i) * cos((robot->angle) * PI / 180));
        xTL = (int)xDir;
        yTL = (int)yDir;

        SDL_Rect rect = {xTL, yTL, 2, sensor_sensitivity};
        SDL_SetRenderDrawColor(renderer, 80 + (20 * (5 - i)), 80 + (20 * (5 - i)), 80 + (20 * (5 - i)), 255);
        SDL_RenderDrawRect(renderer, &rect);
        SDL_RenderFillRect(renderer, &rect);
    }

    //Front Left Sensor
    for (i = 0; i < 5; i++)
    {
        xDir = round(robotCentreX + (-ROBOT_WIDTH / 2) * cos((robot->angle) * PI / 180) - (-ROBOT_HEIGHT / 2 - SENSOR_VISION + sensor_sensitivity * i) * sin((robot->angle) * PI / 180));
        yDir = round(robotCentreY + (-ROBOT_WIDTH / 2) * sin((robot->angle) * PI / 180) + (-ROBOT_HEIGHT / 2 - SENSOR_VISION + sensor_sensitivity * i) * cos((robot->angle) * PI / 180));
        xTL = (int)xDir;
        yTL = (int)yDir;

        SDL_Rect rect = {xTL, yTL, 2, sensor_sensitivity};
        SDL_SetRenderDrawColor(renderer, 80 + (20 * (5 - i)), 80 + (20 * (5 - i)), 80 + (20 * (5 - i)), 255);
        SDL_RenderDrawRect(renderer, &rect);
        SDL_RenderFillRect(renderer, &rect);
    }
}

void robotMotorMove(struct Robot *robot)
{
    double x_offset, y_offset;
    switch (robot->direction)
    {
    case UP:
        robot->currentSpeed += DEFAULT_SPEED_CHANGE;
        if (robot->currentSpeed > MAX_ROBOT_SPEED)
            robot->currentSpeed = MAX_ROBOT_SPEED;
        break;
    case DOWN:
        robot->currentSpeed -= DEFAULT_SPEED_CHANGE;
        if (robot->currentSpeed < -MAX_ROBOT_SPEED)
            robot->currentSpeed = -MAX_ROBOT_SPEED;
        break;
    case LEFT:
        robot->angle = (robot->angle + 360 - DEFAULT_ANGLE_CHANGE) % 360;
        break;
    case RIGHT:
        robot->angle = (robot->angle + DEFAULT_ANGLE_CHANGE) % 360;
        break;
    }
    robot->direction = 0;
    x_offset = (-robot->currentSpeed * sin(-robot->angle * PI / 180));
    y_offset = (-robot->currentSpeed * cos(-robot->angle * PI / 180));

    robot->true_x += x_offset;
    robot->true_y += y_offset;

    x_offset = round(robot->true_x);
    y_offset = round(robot->true_y);

    robot->x = (int)x_offset;
    robot->y = (int)y_offset;
}

int check = 0;
int turns[5] = {
    LEFT,
    RIGHT,
    RIGHT,
    RIGHT,
    LEFT};
int checkturn = -1;

void robotAutoMotorMove(struct Robot *robot, int front_left_sensor, int front_right_sensor)
{
    //printf("sensors: %d - %d   direction: %d\n", front_left_sensor, front_right_sensor, robot->direction);
    if ((front_left_sensor == 0) && (front_right_sensor == 0))
    {
        if (robot->currentSpeed < 2)
        {
            robot->direction = UP;
            check = 1;
        }
    }
    else if ((robot->currentSpeed > 0) && ((front_left_sensor == 1) || (front_right_sensor == 1)))
    {
        robot->direction = DOWN;
    }
    else if ((robot->currentSpeed == 0) && ((front_left_sensor == 1) || (front_right_sensor == 1)))
    {

        if (check == 1)
        {
            checkturn += 1;
        }
        robot->direction = turns[checkturn];
        check = 0;
    }
    else if ((robot->currentSpeed == 0) && ((front_left_sensor == 1) || (front_right_sensor == 0)))
    {
        robot->direction = RIGHT;
    }
    else if ((robot->currentSpeed == 0) && ((front_left_sensor == 0) || (front_right_sensor == 1)))
    {
        robot->direction = LEFT;
    }

    printf("DIRECTION: %d, checkturn: %d, sensors: %d - %d \n", robot->direction, checkturn, front_left_sensor, front_right_sensor);
}

//------------------------------------------------------- Robot ------------------------------------------------------

//------------------------------------------------------- Wall ------------------------------------------------------

void wallSetPosition(struct Wall *wall, int x, int y, int width, int height)
{
    wall->x = x;
    wall->y = y;
    wall->width = width;
    wall->height = height;
}

void wallUpdate(SDL_Renderer *renderer, struct Wall *wall)
{
    SDL_Rect rect = {wall->x, wall->y, wall->width, wall->height};
    SDL_SetRenderDrawColor(renderer, 207, 99, 85, 255);
    SDL_RenderFillRect(renderer, &rect);
    SDL_RenderDrawRect(renderer, &rect);
}

//insert link at the first location
void insertFirstWall(struct Wall_collection **head, int key, struct Wall *wall)
{
    //create a link
    struct Wall_collection *link = (struct Wall_collection *)malloc(sizeof(struct Wall_collection));

    link->key = key;
    link->wall = *wall;

    //point it to old first node
    link->next = *head;

    //point first to new first node
    *head = link;
}

void insertAndSetFirstWall(struct Wall_collection **head, int key, int x, int y, int width, int height)
{
    //create a link
    struct Wall *wall = (struct Wall *)malloc(sizeof(struct Wall));
    wallSetPosition(wall, x, y, width, height);
    insertFirstWall(head, key, wall);
}

void updateAllWalls(struct Wall_collection *head, SDL_Renderer *renderer)
{
    struct Wall_collection *ptr = head;

    //start from the beginning
    while (ptr != NULL)
    {
        //printf("(%d)",ptr->key);
        wallUpdate(renderer, &ptr->wall);
        ptr = ptr->next;
    }
}

//------------------------------------------------------- Wall ------------------------------------------------------

int main(int argc, char *argv[])
{

    SDL_Window *window;
    SDL_Renderer *renderer;

    if (SDL_Init(SDL_INIT_VIDEO) < 0)
    {
        return 1;
    }

    window = SDL_CreateWindow("Robot Maze", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, OVERALL_WINDOW_WIDTH, OVERALL_WINDOW_HEIGHT, SDL_WINDOW_OPENGL);
    window = SDL_CreateWindow("Robot Maze", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, OVERALL_WINDOW_WIDTH, OVERALL_WINDOW_HEIGHT, SDL_WINDOW_OPENGL);
    renderer = SDL_CreateRenderer(window, -1, 0);

    struct Robot robot;
    struct Wall_collection *head = NULL;
    struct Wall_collection *checkpoints = NULL;
    int front_left_sensor, front_right_sensor = 0;
    clock_t start_time, end_time;
    int msec;

    // SETUP MAZE
    // You can create your own maze here. line of code is adding a wall.
    // You describe position of top left corner of wall (x, y), then width and height going down/to right
    // Relative positions are used (OVERALL_WINDOW_WIDTH and OVERALL_WINDOW_HEIGHT)
    // But you can use absolute positions. 10 is used as the width, but you can change this.
    insertAndSetFirstWall(&head, 1, OVERALL_WINDOW_WIDTH / 2, OVERALL_WINDOW_HEIGHT / 2, 10, OVERALL_WINDOW_HEIGHT / 2);
    insertAndSetFirstWall(&head, 2, OVERALL_WINDOW_WIDTH / 2 - 100, OVERALL_WINDOW_HEIGHT / 2 + 100, 10, OVERALL_WINDOW_HEIGHT / 2 - 100);
    insertAndSetFirstWall(&head, 3, OVERALL_WINDOW_WIDTH / 2 - 250, OVERALL_WINDOW_HEIGHT / 2 + 100, 150, 10);
    insertAndSetFirstWall(&head, 4, OVERALL_WINDOW_WIDTH / 2 - 150, OVERALL_WINDOW_HEIGHT / 2, 150, 10);
    insertAndSetFirstWall(&head, 5, OVERALL_WINDOW_WIDTH / 2 - 250, OVERALL_WINDOW_HEIGHT / 2 - 200, 10, 300);
    insertAndSetFirstWall(&head, 6, OVERALL_WINDOW_WIDTH / 2 - 150, OVERALL_WINDOW_HEIGHT / 2 - 100, 10, 100);
    insertAndSetFirstWall(&head, 7, OVERALL_WINDOW_WIDTH / 2 - 250, OVERALL_WINDOW_HEIGHT / 2 - 200, 450, 10);
    insertAndSetFirstWall(&head, 8, OVERALL_WINDOW_WIDTH / 2 - 150, OVERALL_WINDOW_HEIGHT / 2 - 100, 250, 10);
    insertAndSetFirstWall(&head, 9, OVERALL_WINDOW_WIDTH / 2 + 200, OVERALL_WINDOW_HEIGHT / 2 - 200, 10, 300);
    insertAndSetFirstWall(&head, 10, OVERALL_WINDOW_WIDTH / 2 + 100, OVERALL_WINDOW_HEIGHT / 2 - 100, 10, 300);
    insertAndSetFirstWall(&head, 11, OVERALL_WINDOW_WIDTH / 2 + 100, OVERALL_WINDOW_HEIGHT / 2 + 200, OVERALL_WINDOW_WIDTH / 2 - 100, 10);
    insertAndSetFirstWall(&head, 12, OVERALL_WINDOW_WIDTH / 2 + 200, OVERALL_WINDOW_HEIGHT / 2 + 100, OVERALL_WINDOW_WIDTH / 2 - 100, 10);

    setup_robot(&robot);

    updateAllWalls(head, renderer);

    SDL_Event event;
    while (!done)
    {
        SDL_SetRenderDrawColor(renderer, 200, 200, 200, 255);
        SDL_RenderClear(renderer);

        //Move robot based on user input commands/auto commands
        if (robot.auto_mode == 1)
            robotAutoMotorMove(&robot, front_left_sensor, front_right_sensor);

        robotMotorMove(&robot);

        //Check if robot reaches endpoint. and check sensor values
        if (checkRobotReachedEnd(&robot, OVERALL_WINDOW_WIDTH, OVERALL_WINDOW_HEIGHT / 2 + 100, 10, 100))
        {
            end_time = clock();
            msec = (end_time - start_time) * 1000 / CLOCKS_PER_SEC;
            robotSuccess(&robot, msec);
        }
        else if (checkRobotHitWalls(&robot, head))
            robotCrash(&robot);
        //Otherwise compute sensor information
        else
        {
            front_left_sensor = checkRobotSensorFrontLeftAllWalls(&robot, head);
            // if (front_left_sensor > 0)
            //     printf("Getting close on the left. Score = %d\n", front_left_sensor);

            front_right_sensor = checkRobotSensorFrontRightAllWalls(&robot, head);
            // if (front_right_sensor > 0)
            //     printf("Getting close on the right. Score = %d\n", front_right_sensor);
        }
        robotUpdate(renderer, &robot);
        updateAllWalls(head, renderer);

        // Check for user input
        SDL_RenderPresent(renderer);
        while (SDL_PollEvent(&event))
        {
            if (event.type == SDL_QUIT)
            {
                done = 1;
            }
            const Uint8 *state = SDL_GetKeyboardState(NULL);
            if (state[SDL_SCANCODE_UP] && robot.direction != DOWN)
            {
                robot.direction = UP;
            }
            if (state[SDL_SCANCODE_DOWN] && robot.direction != UP)
            {
                robot.direction = DOWN;
            }
            if (state[SDL_SCANCODE_LEFT] && robot.direction != RIGHT)
            {
                robot.direction = LEFT;
            }
            if (state[SDL_SCANCODE_RIGHT] && robot.direction != LEFT)
            {
                robot.direction = RIGHT;
            }
            if (state[SDL_SCANCODE_SPACE])
            {
                setup_robot(&robot);
            }
            if (state[SDL_SCANCODE_RETURN])
            {
                robot.auto_mode = 1;
                start_time = clock();
            }
        }

        SDL_Delay(120);
    }
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    printf("DEAD\n");
}
