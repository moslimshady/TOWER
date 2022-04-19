#include "NAVCOMP_MODULE.h"
#include "global.h"

void NAVCOMP_MODULE()
{
    if (receivedFullCommand == 1)
    {
        if (strncmp((char *)rc.command1, "mv", 2) == 0)
        {
            if (strncmp((char *)rc.command2, "sp", 2) == 0)
            {
                /* code */
                desired_speed = rc.parameter1;
                taskCompleted = 1;
            }
            else if (strncmp((char *)rc.command2, "ds", 2) == 0)
            {
                moveStop();
                desired_steps = 4096 * rc.parameter1 / (M_PI * EMMA_WHEEL_DIA);
                while (stepCompleted == 0)
                {
                    comm = 2;
                }
                comm = 1;
                moveStop();
                taskCompleted = 1;
            }
            else if (strncmp((char *)rc.command2, "wr", 2) == 0)
            {
                if (rc.parameter1 >= 0 && rc.parameter2 >= 0 && rc.parameter1 < 7 && rc.parameter2 < 7)
                    exploreLabyrinth(rc.parameter1, rc.parameter2);
                else
                    printToHterm(sprintf(uart_buf, "You've entered invalid parameters for mv wr."));
                taskCompleted = 1;
            }
            else if (strncmp((char *)rc.command2, "st", 2) == 0)
            {
                comm = 1;
                taskCompleted = 1;
                /* code */
            }
            else if (strncmp((char *)rc.command2, "rt", 2) == 0)
            {

                desired_speed_left = abs(rc.parameter1);
                desired_speed_right = abs(rc.parameter2);
                setSpeedLeftMotor();
                setSpeedRightMotor();
                if (rc.parameter1 > 0 && rc.parameter2 < 0)
                {
                    desired_angle = -720;
                    while (taskCompleted == 0)
                    {
                        comm = 4; // rotate
                    }
                }
                if (rc.parameter2 > 0 && rc.parameter1 < 0)
                {
                    desired_angle = 720;
                    while (taskCompleted == 0)
                    {
                        comm = 4;
                    }
                }
                if (rc.parameter1 >= 0 && rc.parameter2 >= 0)
                {
                    int startTick = HAL_GetTick();
                    while (HAL_GetTick() - startTick > 5000000)
                    {
                        comm = 10; // move forward without a limit.
                    }
                }
                moveStop();
                taskCompleted = 1;
                /* code */
            }
            else if (strncmp((char *)rc.command2, "lr", 2) == 0)
            {
                if (rc.parameter1 != 0 && rc.parameter2 != 0)
                {
                    for (int i = 0; i < rc.parameter1; i++)
                    {
                        if (currentHeading == 4)
                        {
                            currentHeading = 0;
                        }
                        itoa(currentHeading, pathToFollow + i * sizeof(char), 10);
                        // strcat(pathToFollow, ",");
                    }
                    if (abs(currentHeading + 1) >= 4)
                    {
                        currentHeading = 4 - currentHeading;
                    }
                    for (int i = 0; i < rc.parameter2; i++)
                    {
                        itoa(currentHeading + 1, pathToFollow + (i * sizeof(char) + rc.parameter1), 10);
                    }
                    if (abs(currentHeading + 3) >= 4)
                    {
                        currentHeading = 4 - currentHeading;
                    }
                    for (int i = 0; i < rc.parameter2; i++)
                    {
                        itoa(currentHeading + 3, pathToFollow + (i * sizeof(char) + rc.parameter1 + rc.parameter2), 10);
                    }
                    if (abs(currentHeading + 2) >= 4)
                    {
                        currentHeading = 4 - currentHeading;
                    }
                    for (int i = 0; i < rc.parameter1; i++)
                    {
                        itoa(currentHeading + 2, pathToFollow + (i * sizeof(char) + rc.parameter1 + (2 * rc.parameter2)), 10);
                    }
                    followPath(transformStringToPath());
                }
                else
                    printToHterm(sprintf(uart_buf, "Incorrect command for mv lr. Please enter parameters."));
                taskCompleted = 1;
            }
            else if (strncmp((char *)rc.command2, "ls", 2) == 0)
            {

                if (initialPositionX < 0 || initialHeading < 0 || initialPositionY < 0)
                {
                    printToHterm(sprintf(uart_buf, "Please enter the initial positions with mz wr and retry mv lr."));
                }
                else
                {
                    if (initialPositionX == 0 && initialPositionY == 0)
                    {
                        strncpy(pathToFollow, pathToCenterFrom00, strlen(pathToCenterFrom00));
                    }
                    else if (initialPositionX == 0 && initialPositionY == 6)
                    {
                        strncpy(pathToFollow, pathToCenterFrom06, strlen(pathToCenterFrom06));
                    }
                    else if (initialPositionX == 6 && initialPositionY == 0)
                    {
                        strncpy(pathToFollow, pathToCenterFrom60, strlen(pathToCenterFrom60));
                    }
                    else if (initialPositionX == 6 && initialPositionY == 6)
                    {
                        strncpy(pathToFollow, pathToCenterFrom66, strlen(pathToCenterFrom66));
                    }
                    printToHterm(sprintf(uart_buf, "pathToFollow %s", pathToFollow));
                    followPath(transformStringToPath());
                }
                taskCompleted = 1;
            }
            else if (strncmp((char *)rc.command2, "ex", 2) == 0)
            {
                exploreLabyrinth(3, 3);
                taskCompleted = 1;
            }
            else if (strncmp((char *)rc.command2, "sc", 2) == 0)
            {
                if (0 < rc.parameter1 && rc.parameter1 < 100)
                {
                    printToHterm(sprintf(uart_buf, "ADJUST_SCALER is now %i percent", rc.parameter1));
                    ADJUST_SCALER = rc.parameter1 / 100;
                }
                taskCompleted = 1;
            }
        }
        else if (strncmp((char *)rc.command1, "mz", 2) == 0)
        {
            if (strncmp((char *)rc.command2, "wr", 2) == 0)
            {

                initialPositionX = rc.parameter1;
                initialPositionY = rc.parameter2;
                initialHeading = rc.parameter3;
                printToHterm(sprintf(uart_buf, "You entered the initial positon and heading as X:%i, Y: %i, Heading: %i", initialPositionX, initialPositionY, initialHeading));
                taskCompleted = 1;
            }
            else if (strncmp((char *)rc.command2, "ps", 2) == 0)
            {
                printToHterm(sprintf(uart_buf, "Current Position in labyrinth, X: %i, Y: %i.", initialPositionX, initialPositionY));
                taskCompleted = 1;
                /* code */
            }
            else if (strncmp((char *)rc.command2, "rd", 2) == 0)
            {
                char *wallsNESW = 0;
                wallsNESW = itoa(labyrinth[currentPositionX][currentPositionY], wallsNESW, 2);
                printToHterm(sprintf(uart_buf, "Current Position in labyrinth, X: %i, Y: %i. Walls NESW %s", currentPositionX, currentPositionY, wallsNESW));
                taskCompleted = 1;
                /* code */
            }
            else if (strncmp((char *)rc.command2, "hd", 2) == 0)
            {
                printToHterm(sprintf(uart_buf, "Current Heading, where 0..3 correspond to N, E, S, W: %i", currentHeading));
                taskCompleted = 1;
                /* code */
            }
        }
        printToHterm(sprintf(uart_buf, "You selected %s %s", rc.command1, rc.command2));

        if (taskCompleted == 1)
        {
            printToHterm(sprintf(uart_buf, "Task completed."));
            clearRemoteCommandStruct();
            taskCompleted = 0;
            receivedFullCommand = 0;
        } 
    }
}

int transformStringToPath()
{
    int turn = 0;
    char c[1];
    int len = 0;
    strncat(pathToFollow, "x", 1);
    while(strncmp(&pathToFollow[len], "x", 1))
    {
        len++;
    }
    int j = 0;
    for (int i = 0; i < len; i++) // i is for turnArray, j is for pathToFollow
    {

        strncpy(&c[0], pathToFollow + j, 1);
        turn = currentHeading - atoi(c);
        // positive turn is left (anticlockwise angle), currentHeading is 0123 NESW (clockwise cardinal dir.)
        currentHeading = currentHeading - turn;
        if (abs(currentHeading) >= 4)
        {
            if (currentHeading < 0)
            {
                currentHeading = currentHeading + 4;
            }
            else
            {
                currentHeading = currentHeading - 4;
            }
        }
        if (turn != 0)
        {
            turnArray[i] = turn;
            len++;
        }
        else
        {
            turnArray[i] = 0;
            j++;
        }
    }

    strncpy(pathToFollow, "0", 1);
    return len;
}
void followPath(int pathLength)
{
    // double totalRequiredForwardLeg = 0;
    // int numberOfCellsTravelledLeg = 0;

    double requiredForward = 0;
    for (int i = 0; i < pathLength; i++)
    {

        desired_angle = turnArray[i] * 90;
        if (desired_angle == 0)
        {
            // numberOfCellsTravelledLeg++;
            stepCompleted = 0;
            for (int j = 0; j < ADJUSTS_PER_CELL; j++)
            {
                requiredForward = ADJUST_SCALER * adjustPath();
                desired_steps = 4096 * requiredForward / (M_PI * EMMA_WHEEL_DIA);
                // totalRequiredForwardLeg = totalRequiredForwardLeg + requiredForward;

                // printToHterm(sprintf(uart_buf, "Total Required Forward Leg: %i mm", (int)totalRequiredForwardLeg));
                stepCompleted = 0;
                while (stepCompleted == 0)
                {
                    comm = 2;
                }
                totalDistanceInMillimeters = totalDistanceInMillimeters + (desired_steps * (M_PI * EMMA_WHEEL_DIA) / 4096);
                updateLocation();
                measureSurroundings();
                printUsValues();
                printWallValues();
                // adjustYaw();
                moveStop();
            }
            desired_angle = turnArray[i];
        }

        else
        {
            // numberOfCellsTravelledLeg = 0;
            // totalRequiredForwardLeg = 0;
            stepCompleted = 0;
            printToHterm(sprintf(uart_buf, "Turning %i", (int)desired_angle));
            while (stepCompleted == 0)
            {
                comm = 4;
            }
            measureSurroundings();
            printUsValues();
            printWallValues();
            moveStop();
        }
    }
    taskCompleted = 1;
}
void adjustYaw()
{
    stepCompleted = 0;
    desired_angle = 90 - currentYaw;
    if (fabs(desired_angle) > MAX_YAW_ADJUST)
        return;
    while (stepCompleted == 0)
    {
        comm = 4;
    }
    printToHterm(sprintf(uart_buf, "Adjusted yaw by %i.%i degrees", (int)desired_angle, (int)(abs(desired_angle) - floor(abs(desired_angle)))));
}

void updateLocation()
{
    switch (currentHeading)
    {
    case 0:
        currentPositionY--;
        break;
    case 1:
        currentPositionX++;
        break;
    case 2:
        currentPositionY++;
        break;
    case 3:
        currentPositionX--;
        break;
    case -1:
        currentPositionX--;
        break;
    case -2:
        currentPositionY++;
        break;
    case -3:
        currentPositionX++;
        break;
    default:
        break;
    }
    // printToHterm(sprintf(uart_buf, "Current coordinates on labyrinth: X: %i, Y: %i", currentPositionX, currentPositionY));
}

double adjustPath()
{
    double deltaForward = 0;
    double requiredLateral = 0;
    double requiredForward = CELL_LENGTH / ADJUSTS_PER_CELL;
    requiredYaw = 0;
    int cellWidth = CELL_LENGTH - WALL_WIDTH;
    if ((dis_L > 100) ^ (dis_R > 100))
    {
        dis_L = dis_L > dis_R ? (cellWidth - SENSOR_WIDTH - dis_R) : dis_L;
        dis_R = dis_L > dis_R ? (dis_R) : (cellWidth - SENSOR_WIDTH - dis_L);
    }
    else if (dis_L > 100 && dis_R > 100)
    {
        requiredForward = CELL_LENGTH / ADJUSTS_PER_CELL;
        requiredYaw = 0;
        // if (150 < dis_X && dis_X < 300)
        //     return CELL_LENGTH - (WALL_WIDTH / 2) - DEADEND_ADJUST;
        // else
        // return CELL_LENGTH;
        return requiredForward;
    }
    if (dis_X < FWD_MEASURE_LIMIT)
    {
        deltaForward = (((int)((dis_X + (WALL_WIDTH / 2))) % CELL_LENGTH) - FWD_IDEAL) / ADJUSTS_PER_CELL;
        requiredForward = CELL_LENGTH / ADJUSTS_PER_CELL + deltaForward;
        requiredLateral = (dis_L > dis_R ? (dis_L - LAT_IDEAL) : dis_R - LAT_IDEAL) / ADJUSTS_PER_CELL;

        if (dis_L == dis_R)
        {
            requiredLateral = 0;
        }
        int multiplier = (dis_L > dis_R ? 1 : -1);
        requiredYaw = tan(requiredLateral / requiredForward) * 180 / M_PI * multiplier;
        if (requiredYaw != 0.0 && abs((int)requiredYaw) < MAX_YAW_ADJUST)
        {
            printToHterm(sprintf(uart_buf, "Adjusting Path by %i.%i degrees", (int)requiredYaw, (int)(100 * (fabs(requiredYaw) - floor(abs(requiredYaw))))));
            desired_angle = requiredYaw;
            stepCompleted = 0;
            while (stepCompleted == 0)
            {
                comm = 4;
            }
        }
        else
            requiredYaw = 0;
    }
    // if (dis_X < 300)
    //     return fabs(dis_X - DEADEND_ADJUST);
    if ((CELL_LENGTH - MAX_FWD_ADJUST) / ADJUSTS_PER_CELL < requiredForward && requiredForward < (CELL_LENGTH + MAX_FWD_ADJUST) / ADJUSTS_PER_CELL)
        return requiredForward;
    else
        return CELL_LENGTH / ADJUSTS_PER_CELL;
}

void exploreLabyrinth(int destX, int destY)
{
    // double totalRequiredForwardLeg = 0;
    // int numberOfCellsTravelledLeg = 0;
    while (!(currentPositionX == destX && currentPositionY == destY))
    {
        double requiredForward = 0;
        int frontWall = 0;
        int rightWall = 0;
        int leftWall = 0;
        rightWall = dis_R > MAX_LAT_WALL_DIST ? 0 : 1;
        leftWall = dis_L > MAX_LAT_WALL_DIST ? 0 : 1;
        frontWall = dis_X > MAX_FRNT_WALL_DIST ? 0 : 1;
        if (rightWall == 0)
            desired_angle = -90;
        else if (frontWall == 0)
            desired_angle = 0;
        else if (leftWall == 0)
            desired_angle = 90;

        if (desired_angle != 0)
        {
            stepCompleted = 0;
            currentHeading = currentHeading - (desired_angle / 90);
            if (abs(currentHeading) > 3)
            {
                if (currentHeading < 0)
                    currentHeading = currentHeading + 4;
                else
                    currentHeading = currentHeading - 4;
            }
            while (stepCompleted != 0)
            {
                comm = 4;
            }
        }
        if (desired_angle == 0)
        {
            updateLocation();
            for (int iter = 0; iter < ADJUSTS_PER_CELL; iter++)
            {
                requiredForward = adjustPath();
                desired_steps = 4096 * requiredForward / (M_PI * EMMA_WHEEL_DIA);
                stepCompleted = 0;
                while (stepCompleted == 0)
                {
                    comm = 2;
                }
                measureSurroundings();
                printUsValues();
                printWallValues();
            }
        }
        printToHterm(sprintf(uart_buf, "X: %i, Y:%i, Heading: %i", currentPositionX, currentPositionY, currentHeading));
    }
    taskCompleted = 1;
}