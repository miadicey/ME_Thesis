#include <stdio.h> // Stdio defines printf command.
#include <stdint.h> // Standard int will define the datatypes we are using e.g. int32
#include <stdbool.h> // Standard bool contains nice boolean definitions

void findQ(void)
{
    uint32_t i = 0;
//    uint32_t count = 0;
    char c;

    do {
        c = getchar();
        i++;
    } while (c != 'q');
    printf("%d\n", i);
}

int main(void)
{
    findQ();
}