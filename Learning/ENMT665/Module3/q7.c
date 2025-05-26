#include <stdio.h> // Stdio defines printf command.
#include <stdint.h> // Standard int will define the datatypes we are using e.g. int32
#include <stdbool.h> // Standard bool contains nice boolean definitions



int main(void)
{
    int32_t num1;
    int32_t num2;

    scanf("%d %d", &num1, &num2);
    printf("%d\n", num1+num2);
}
