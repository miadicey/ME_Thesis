#include <stdio.h> // Stdio defines printf command.
#include <math.h> //Defines parameters for sqrts
#include <stdint.h> // Standard int will define the datatypes we are using e.g. int32
#include <stdbool.h> // Standard bool contains nice boolean definitions


void printRoots(float a, float b, float c)
{
    double x1;
    double x2;

    if (a == 0.0) {
        printf("Not a quadratic\n");
    } else if  ((b*b - 4*a*c) < 0) {
        printf("Imaginary roots\n");
    } else {
        x1 = (-b + sqrt(b*b - 4*a*c))/(2*a);
        x2 = (-b - sqrt(b*b - 4*a*c))/(2*a);
        printf("Roots are %.4f and %.4f\n", x1<= x2 ? x1:x2, x1<=x2 ? x2:x1);
    }

}

int main(void)
{
    printRoots(1, 0, -1);
}