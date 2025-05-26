#include <stdio.h> // Stdio defines printf command.
#include <stdint.h> // Standard int will define the datatypes we are using e.g. int32
#include <stdbool.h> // Standard bool contains nice boolean definitions

void printViaPtr(int16_t* intPtr)
{
    printf("%d\n", *intPtr);
}

void print2Ints(int16_t number1, int16_t number2)
{
    int16_t* ptr;
    ptr=&number1;
    printViaPtr(ptr);    
    ptr=&number2;
    printViaPtr(ptr);

}

int main(void)
{
    print2Ints(11, -93);
}