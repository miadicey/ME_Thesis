#include <stdio.h> // Stdio defines printf command.
#include <stdint.h> // Standard int will define the datatypes we are using e.g. int32
#include <stdbool.h> // Standard bool contains nice boolean definitions

void printSquares(uint32_t n)
{
    uint32_t i;
    for (i=1; i<=n; i++) {
        printf("%d\n", i*i);
    }
}

int main(void)
{
    printSquares(10);
}
