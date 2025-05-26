#include <stdio.h> // Stdio defines printf command.
#include <stdint.h> // Standard int will define the datatypes we are using e.g. int32
#include <stdbool.h> // Standard bool contains nice boolean definitions



int main(void)
{
    int32_t number_a = 5;
    int32_t number_b = 42;
    size_t obj_size;
    int32_t* ptr;   //Pointer of type int32_t, pointr types always have an address, that is its value

    printf("Number A: Value=%d, address=%p\n", number_a, &number_a);    //& is getting the address of the number
    printf("Number B: Value=%d, address=%p\n", number_b, &number_b);

    ptr =&number_a;
    printf("Pointer: Value=%p, address=%p, dereference value=%d\n", ptr, &ptr, *ptr); //value of pointer is an address, address of pointer is an address, dereference value points to a so is a value

    ptr =&number_b;
    *ptr = 7;       //Changes the deference value, which is the value of b
    printf("Pointer: Value=%p, address=%p, deference value=%d\n", ptr, &ptr, *ptr);
    printf("Number B: Value=%d, address=%p\n", number_b, &number_b);

    printf("Size of Number A: %zu\n", sizeof(number_a));    //A is a 4 byte integer as it is an int 32 so prints 4.

    printf("Difference in addresses %zu\n", &number_a - &number_b);

    obj_size = sizeof(float);
    printf("The object size is %zu bytes\n", obj_size);
}
