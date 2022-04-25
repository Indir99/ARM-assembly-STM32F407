#include <stdio.h>
#include <string.h>

int main()
{
    char *message = "HOW ARE YOU?";
    int i = 0;
    while (message[i] != '\0')
    {
        i++;
    }
    printf("Counter value: %d ", i);
    printf("\n");

    return 0;
}