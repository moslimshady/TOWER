#include <stdio.h>
#include <stdlib.h>


// This function accepts an array of uin8_t to char*. 
// Used to stringcmp rx_data with expected commands 
typedef unsigned char uint8_t;

char *convert_uintArray_to_charPtr(uint8_t *rx_data)
{
  char* temp = malloc(sizeof(rx_data));
  int i=0;
  int len = sizeof(rx_data);
  char* temp2= temp;
  for(i=0;i<len;i++){
    *temp2 = rx_data[i];
    temp2++;
  }
  puts(temp);
  printf("here\r\n");
  printf("temp: %s", (temp));

  puts(temp);

  return temp;
}