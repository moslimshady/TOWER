#include "DIST_MODULE.h"
#include "global.h"

void DIST_MODULE()
{
  if (receivedFullCommand == 1)
  {

    if (strncmp((char *)rc.command1, "tm", 2) == 0)
    {
      comm = 1;
      if (strncmp((char *)rc.command2, "ds", 2) == 0)
      {
        printToHterm(sprintf(uart_buf, "You selected tm ds"));
        DIST_MODULE_TMDS();
        taskCompleted = 1;
      }
      else if (strncmp((char *)rc.command2, "od", 2) == 0)
      {
        printToHterm(sprintf(uart_buf, "You selected tm od"));
        printToHterm(sprintf(uart_buf, "Total Distance travelled by EMMA: %i", totalDistanceInMillimeters));
        taskCompleted = 1;
        /* code */
      }
      else if (strncmp((char *)rc.command2, "vc", 2) == 0)
      {
        printToHterm(sprintf(uart_buf, "You selected tm vc"));
        printToHterm(sprintf(uart_buf, "Vector from start position X: %i, Y: %i", initialPositionX - currentPositionX, initialPositionY - currentPositionY));
        taskCompleted = 1;
        /* code */
      }
      else if (strncmp((char *)rc.command2, "hd", 2) == 0)
      {
        printToHterm(sprintf(uart_buf, "You selected tm hd"));
        currentYaw = 0;
        measureSurroundings();
        printToHterm(sprintf(uart_buf, "Current attitude/yaw in degrees: %i", (int)currentYaw));
        // printToHterm(sprintf(uart_buf, "Current attitude/yaw in degrees: %i", (int)currentYaw));
        taskCompleted = 1;
        /* code */
      }
      if (taskCompleted == 1)
      {
        printToHterm(sprintf(uart_buf, "Task completed."));
        clearRemoteCommandStruct();
        receivedFullCommand = 0;
        taskCompleted = 0;
      }
    }
  }
}

void DIST_MODULE_TMDS()
{
  int printCounter = 0;
  while (printCounter < PRINTCOUNTER)
  {
    // Prints out the telemetry values this many times.
    int readCounter = 0;
    int startTickPrint = HAL_GetTick();
    int startTickRead =0;
    for (readCounter = 0; readCounter < VECTOR_SIZE_FOR_MODE; readCounter++)
    {
      startTickRead = HAL_GetTick();

      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_SET);

      while (HAL_GetTick() - startTickRead <= 10)
        ;

      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_RESET);
      printUsValues();
      updateDistanceMatrix(readCounter);
    }
    dis_L = (float)modeOfVector(0);
    dis_X = (float)modeOfVector(1);
    dis_R = (float)modeOfVector(2);
    while (HAL_GetTick() - startTickPrint <= 1000)
      ;
    startTickPrint = HAL_GetTick();
    printUsValues();
    printCounter++;
  }
}
void measureSurroundings()
{
  for (int readCounter = 0; readCounter < VECTOR_SIZE_FOR_MODE; readCounter++)
  {
    int startTick = HAL_GetTick();
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_SET);

    while (HAL_GetTick() - startTick < 10)
      ;
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_RESET);
    updateDistanceMatrix(readCounter);
  }
  dis_L = (float)modeOfVector(0);
  dis_X = (float)modeOfVector(1);
  dis_R = (float)modeOfVector(2);

  updateLatestTwoDistances();
  calculateYaw();
}
void updateDistanceMatrix(uint8_t indexOfVector)
{
  distanceMatrix[indexOfVector][0] = (int)dis_L;
  distanceMatrix[indexOfVector][1] = (int)dis_X;
  distanceMatrix[indexOfVector][2] = (int)dis_R;
}
void printUsValues()
{
  printToHterm(sprintf(uart_buf, "Vorne: %u mm, Links: %u mm, Rechts: %u mm", (uint)dis_X, (uint)dis_L, (uint)dis_R));
}
void printWallValues()
{
  int leftWall = 0;
  int rightWall = 0;
  int frontWall = 0;

  if(dis_X<100)
    frontWall = 1;
  if(dis_L <100)
    leftWall = 1;
  if(dis_R<100)
    rightWall = 1;
  printToHterm(sprintf(uart_buf, "Front Wall: %i, Left Wall: %i, Right Wall: %i", frontWall, leftWall, rightWall));

}
void DIST_MODULE_MEASURE(uint16_t GPIO_Pin)
{
  // Calculate the distance between the sensors and obstacles using time pulse width
  // if-statement to filter interrupt events

  // Frontal US Sensor
  if (GPIO_Pin == GPIO_PIN_9)
  {

    if (Is_First_Captured_X == 0) // if the first value is not captured
    {
      t0_X = __HAL_TIM_GET_COUNTER(&htim1); // read the first value
      Is_First_Captured_X = 1;              // set the first captured as true
    }

    else if (Is_First_Captured_X == 1) // if the first is already captured
    {
      t1_X = __HAL_TIM_GET_COUNTER(&htim1); // read second value

      if (t1_X > t0_X)
      {
        Diff_X = (t1_X - t0_X);
      }

      else if (t0_X > t1_X)
      {
        Diff_X = ((65535 - t0_X) + t1_X);
      }

      dis_X = Diff_X * 0.017 * 10; // cm to mm -> *10
      Is_First_Captured_X = 0;     // set it back to false
    }
  }
  // Left US Sensor
  else if (GPIO_Pin == GPIO_PIN_12)
  {

    if (Is_First_Captured_L == 0) // if the first value is not captured
    {
      t0_L = __HAL_TIM_GET_COUNTER(&htim1); // read the first value
      Is_First_Captured_L = 1;              // set the first captured as true
    }

    else if (Is_First_Captured_L == 1) // if the first is already captured
    {
      t1_L = __HAL_TIM_GET_COUNTER(&htim1); // read second value

      if (t1_L > t0_L)
      {
        Diff_L = (t1_L - t0_L);
      }

      else if (t0_L > t1_L)
      {
        Diff_L = ((65535 - t0_L) + t1_L);
      }

      dis_L = Diff_L * 0.017 * 10; // cm to mm -> *10
      Is_First_Captured_L = 0;     // set it back to false
    }
  }
  // Right US Sensor
  else if (GPIO_Pin == GPIO_PIN_15)
  {

    if (Is_First_Captured_R == 0) // if the first value is not captured
    {
      t0_R = __HAL_TIM_GET_COUNTER(&htim1); // read the first value
      Is_First_Captured_R = 1;              // set the first captured as true
    }

    else if (Is_First_Captured_R == 1) // if the first is already captured
    {
      t1_R = __HAL_TIM_GET_COUNTER(&htim1); // read second value

      if (t1_R > t0_R)
      {
        Diff_R = (t1_R - t0_R);
      }

      else if (t0_R > t1_R)
      {
        Diff_R = ((65535 - t0_R) + t1_R);
      }

      dis_R = Diff_R * 0.017 * 10; // cm to mm -> *10
      Is_First_Captured_R = 0;     // set it back to false
    }
  }
}

void updateLatestTwoDistances()
{
  for (int i = 0; i < 3; i++)
  {
    latestTwoDistanceMeasurements[0][i] = latestTwoDistanceMeasurements[1][i];
  }
  latestTwoDistanceMeasurements[1][0] = dis_L;
  latestTwoDistanceMeasurements[1][1] = dis_X;
  latestTwoDistanceMeasurements[1][2] = dis_R;
}
void calculateYaw()
{
  if (latestTwoDistanceMeasurements[0][2] > latestTwoDistanceMeasurements[1][2])
  {
    currentYaw = 90 - (atan(CELL_LENGTH / (latestTwoDistanceMeasurements[0][2] - latestTwoDistanceMeasurements[1][2])) * 180 / M_PI);
  }
  else if (latestTwoDistanceMeasurements[1][2] > latestTwoDistanceMeasurements[0][2])
  {
    currentYaw = 180 - (atan(CELL_LENGTH / (latestTwoDistanceMeasurements[1][2] - latestTwoDistanceMeasurements[0][2])) * 180 / M_PI);
  }
}