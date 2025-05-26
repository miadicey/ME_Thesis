#include <stdio.h> // Stdio defines printf command.
#include <stdint.h> // Standard int will define the datatypes we are using e.g. int32
#include <stdbool.h> // Standard bool contains nice boolean definitions

void printPrimesInRange(uint32_t n1, uint32_t n2)
{
    bool isPrime = true;
    uint32_t i = 0;
    uint32_t j = 0;

    for (i=n1; i<=n2; i++) {
        for (j=2; j<=i-1; j++) {
            if (i%j == 0) {
                isPrime = false;
            }
        }
        if (isPrime) {
            printf("%d\n", i);
        } else {
            isPrime = true;
        }
    }
}

int main(void)
{
    printPrimesInRange(2, 40);
}
