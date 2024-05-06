#include <stdio.h>

void decToBinary(int n, char *binaryArray) {
    int binaryNum[8],i;
    for (i = 7; i >= 0; i--) {
        binaryNum[i] = n%2;
        n = n / 2;
    }
    for (i = 0; i < 8; i++) {
        binaryArray[i] = binaryNum[i] + '0'; // Convert binary digit to character ('0' or '1')
    }
}

void intToChar(int num, char *str) {
    sprintf(str, "%d", num);
}

void floatToChar(float num, char *str) {
    sprintf(str, "%.2f", num);
}




