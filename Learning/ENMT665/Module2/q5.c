#include <stdio.h> // Stdio defines printf command.
#include <stdint.h> // Standard int will define the datatypes we are using e.g. int32
#include <stdbool.h> // Standard bool contains nice boolean definitions

uint8_t intLog2(uint32_t value)
{
    uint32_t count = 0;
    while (value > 1) {
        value = value / 2;
        count++;
    }
    return count;
}

int main(void)
{
    printf("%d\n", intLog2(3));
}
