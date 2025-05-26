#include <stdio.h> // Stdio defines printf command.
#include <stdint.h> // Standard int will define the datatypes we are using e.g. int32
#include <stdbool.h> // Standard bool contains nice boolean definitions



int32_t accumulator(int32_t value)
{
    static int32_t accumulator_sum = 0;
    accumulator_sum+= value;
    return accumulator_sum;
}

int main(void)
{
    accumulator(3);
    accumulator(2);
    printf("%d\n", accumulator(5));
}
