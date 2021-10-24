#include "stdio.h"
#include "stdlib.h"
#include "SDL2/SDL.h"
#include "SDL2_gfxPrimitives.h"
#include "time.h"

#include "formulas.h"
#include "wall.h"
#include "robot.h"

int done = 0;

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
            if (front_left_sensor > 0)
                printf("Getting close on the left. Score = %d\n", front_left_sensor);

            front_right_sensor = checkRobotSensorFrontRightAllWalls(&robot, head);
            if (front_right_sensor > 0)
                printf("Getting close on the right. Score = %d\n", front_right_sensor);
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
