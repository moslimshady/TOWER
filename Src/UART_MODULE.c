#include "UART_MODULE.h"
#include "global.h"

void UART_MODULE()
{

    stringContainsCRLF = NULL;
    stringContainsCRLF = strstr(data_buffer, "\r\n");
    char *nextToken = 0;
    int tokenCount = 0;

    if (stringContainsCRLF != NULL)
    {
        receivedFullCommand = 1;
        strncpy(data_buffer_temp, data_buffer, sizeof data_buffer);
        nextToken = strtok(data_buffer_temp, ",; ");
        memset(data_buffer, 0, sizeof data_buffer);
    }
    while (nextToken != NULL && receivedFullCommand == 1)
    {

        switch (tokenCount)
        {
        case 0:
            rc.command1[0] = (char)nextToken[0];
            rc.command1[1] = (char)nextToken[1];
            break;
        case 1:
            rc.command2[0] = (char)nextToken[0];
            rc.command2[1] = (char)nextToken[1];
            break;
        case 2:
            rc.parameter1 = atoi(nextToken);
            break;
        case 3:
            rc.parameter2 = atoi(nextToken);
            break;
        case 4:
            rc.parameter3 = atoi(nextToken);
            break;
        case 5:
            rc.parameter4 = atoi(nextToken);
            break;
        case 6:
            rc.parameter5 = atoi(nextToken);
            break;
        case 7:
            rc.parameter6 = atoi(nextToken);
            break;
        default:
            printToHterm(sprintf(uart_buf, "Too many parameters, maximum in expected format: xx xx <int1> <int2> <int3> <int4> <int5> <int6>"));
            break;
        }
        // printf ("%s\n",nextToken);
        nextToken = strtok(NULL, ",; ");
        tokenCount++;
    }
    if (receivedFullCommand == 1)
    {
        clearBuffer();
    }
}