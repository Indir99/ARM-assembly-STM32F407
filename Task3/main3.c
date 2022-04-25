#include <stdio.h>

int main()
{
    int x, y;
    printf("Enter the value for x: ");
    scanf("%d", &x);
    if (x > 10)
    {
        y = 11;
    }
    else if (x < 10)
    {
        y = 9;
    }
    else
    {
        y = 10;
    }
    printf("Value y: %d", y);
    printf("\n");

    return 0;
}