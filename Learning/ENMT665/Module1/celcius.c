/* Converts from Farenheight to Celsius */

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h> //without this exit success doesnt work, as it is a macro defined in standard lib 

#define FREEZING_PT 32 //these are macros, when pp runs the text gets replaced with what comes after
#define SCALE_FACTOR 5/9 //because these are integers, integer division is undertaken

int main(void)
{
    int32_t fahrenheit = 50;
    int32_t celsius = 0;

    celsius = (fahrenheit - FREEZING_PT) * SCALE_FACTOR;

    printf("%d degrees Fahrenheit is equivalent to %d degrees Celsius\n", fahrenheit, celsius);

    return EXIT_SUCCESS;
}