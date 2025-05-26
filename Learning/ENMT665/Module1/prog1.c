/*Your dirst mindless C program*/

// These pull in files with this name. Standard int will define the datatypes we are using e.g. int32. Stdio defines printf command.
#include <stdio.h>
#include <stdint.h>

int main(void)
{
    // First the declarations
    int32_t number1;
    int32_t number2;
    int32_t total;

    // Now some code
    number1 = 10;
    number2 = 20;
    total = number1 + number2;
    printf("The sum of %d and %d is %d\n", number1, number2, total);

    return 0;
}