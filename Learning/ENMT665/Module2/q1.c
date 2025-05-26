#include <stdio.h> // Stdio defines printf command.
#include <stdint.h> // Standard int will define the datatypes we are using e.g. int32
#include <stdbool.h> // Standard bool contains nice boolean definitions

uint32_t countRushes(float screeHeight, float rushHeight, float slideBack) {
    double totHeight = 0.0;
    uint32_t count = 0;

    do {
        if (totHeight >= screeHeight) {
            break;
        }
        totHeight += rushHeight;
        count++;
        if (totHeight >= screeHeight) {
            break;
        } else {
            totHeight -= slideBack;
        }
    } while (totHeight <= screeHeight);
    return count;
}

int main(void)
{
     	

printf("%d\n", countRushes(1.0, 0.5, 0.1));
}
