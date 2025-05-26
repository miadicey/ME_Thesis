#include <stdio.h> // Stdio defines printf command.
#include <stdint.h> // Standard int will define the datatypes we are using e.g. int32
#include <stdbool.h> // Standard bool contains nice boolean definitions

void convertDistance(const double* metres, double* centimetres, double* kilometres)
{
    *centimetres = *metres * 100;
    *kilometres = *metres / 1000;
}

int main(void)
{
    const double metres = 100;
    double centimetres = 0;
    double kilometres = 0;
    convertDistance(&metres, &centimetres, &kilometres);
    printf("%.1e m, %.1e cm, %.1e km\n", metres, centimetres, kilometres);
}
