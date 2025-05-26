#include <stdio.h> // Stdio defines printf command.
#include <stdint.h> // Standard int will define the datatypes we are using e.g. int32
#include <stdbool.h> // Standard bool contains nice boolean definitions

void swap(uint8_t* address1, uint8_t* address2)
{
    uint8_t hold;

    hold = *address1;
    *address1 = *address2;
    *address2 = hold;
}

int main(void)
{
    uint8_t i = 10, j = 20;
    swap(&i, &j);
    printf("%d %d\n", i, j);
}