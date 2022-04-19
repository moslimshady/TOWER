#include "utils.h"
void userDelayUs(uint64_t targetTimeInUs)
{
    uint64_t startTimeUs = sPassed * 1000000 + msPassed * 1000 + usPassed;
    uint64_t tempTargetTime = targetTimeInUs;
    while (sPassed * 1000000 + msPassed * 1000 + usPassed < (startTimeUs + tempTargetTime))
        ;
}
void userDelayMs(uint64_t targetTimeInMs)
{
    uint64_t startTimeUs = sPassed * 1000000 + msPassed * 1000 + usPassed;
    uint64_t tempTargetTime = targetTimeInMs;
    while (sPassed * 1000000 + msPassed * 1000 + usPassed < (startTimeUs + tempTargetTime * 1000))
        ;
}

void userDelayS(uint32_t targetTimeInS)
{
    uint64_t startTimeUs = sPassed * 1000000 + msPassed * 1000 + usPassed;
    uint64_t tempTargetTime = targetTimeInS;
    while (sPassed * 1000000 + msPassed * 1000 + usPassed < (startTimeUs + tempTargetTime * 1000000))
        ;
}

void printTicker()
{
    if (HAL_GetTick() % 1000 == 0)
    {
        printToHterm(sprintf(uart_buf, "Timestamp in ms:, %i", (int)HAL_GetTick()));
        HAL_Delay(1);
    }
}

void clearRemoteCommandStruct()
{
    rc.command1[0] = 0;
    rc.command1[1] = 0;
    rc.command2[0] = 0;
    rc.command2[1] = 0;
    rc.parameter1 = 0;
    rc.parameter2 = 0;
    rc.parameter3 = 0;
    rc.parameter4 = 0;
    rc.parameter5 = 0;
    rc.parameter6 = 0;
}
void clearBuffer()
{
    //memset(data_buffer, 0, strlen((const char*)data_buffer)*sizeof(char));
    //memset(data_buffer_temp, 0, strlen((const char*)data_buffer_temp)*sizeof(char));
    //memset(rx_data, 0, sizeof rx_data)
    for(int i = 0; i<15; i++)
    {
        data_buffer[i] = 0;
        data_buffer_temp[i] = 0;
    }
    rx_data[0] = 0;
}

void floatToInteger(float Distance, int *value1, int *value2)
{ // Transform distance from float to printable integer:
    float tmpVal = (Distance < 0) ? -Distance : Distance;
    int tmpInt1 = tmpVal;                 // Get the integer (678).
    float tmpFrac = tmpVal - tmpInt1;     // Get fraction (0.0123).
    int tmpInt2 = trunc(tmpFrac * 10000); // Turn into integer (123).
    *value1 = tmpInt1;
    *value2 = tmpInt2;
}
void printToHterm(int printInt)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)uart_buf, printInt, 20);
    memset(uart_buf, 0, strlen(uart_buf));
    printInt = sprintf(uart_buf, "\r\n");
    HAL_UART_Transmit(&huart1, (uint8_t *)uart_buf, printInt, 20);
    memset(uart_buf, 0, strlen(uart_buf));
}
uint16_t modeOfVector(uint8_t indexOfMatrix)
{
    // Store count of each element
    // of input array
    uint8_t count[VECTOR_SIZE_FOR_MODE];
    uint8_t iter = 0;
    for (iter = 0; iter < VECTOR_SIZE_FOR_MODE; iter++)
        count[iter] = 0;
    uint8_t currentElement = 0;
    uint8_t compareIter;
    for (iter = 0; iter < VECTOR_SIZE_FOR_MODE; iter++)
    {
        currentElement = distanceMatrix[iter][indexOfMatrix];
        for (compareIter = 0; compareIter < VECTOR_SIZE_FOR_MODE; compareIter++)
        {
            if (distanceMatrix[compareIter][indexOfMatrix] == currentElement)
            {
                count[iter]++;
            }
        }
    }
    // indexOfMode is the index with maximum count
    uint8_t indexOfMode = 0;
    int k = count[0];
    for (int i = 1; i < VECTOR_SIZE_FOR_MODE; i++)
    {
        if (count[i] > k)
        {
            k = count[i];
            indexOfMode = i;
        }
    }
    return distanceMatrix[indexOfMode][indexOfMatrix];
}
char* substring(char *destination, const char *source, int beg, int n)
{
    // extracts `n` characters from the source string starting from `beg` index
    // and copy them into the destination string
    int temp = n;
    while (temp > 0)
    {
        *destination = *(source + beg);
 
        destination++;
        source++;
        temp--;
    }
 
    // null terminate destination string
    *destination = '\0';
 
    // return the destination string
    return destination-n;
}