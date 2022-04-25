#include <stdio.h>

// C implementation

int main()
{
    int a[20];
    int b[20];
    int s = 1;
    for (int i = 0; i < 20; i++)
    {
        a[i] = s;
        s++;
    }
    s = 1;
    for (int i = 0; i < 20; i++)
    {
        b[i] = s;
        s++;
    }
    printf("Array a: ");
    for (int i = 0; i < 20; i++)
    {
        printf(" %d", a[i]);
    }
    printf("\n");
    printf("Array b: ");
    for (int i = 0; i < 20; i++)
    {
        printf(" %d", b[i]);
    }
    printf("\n");
    int sum = 0;
    for (int i = 0; i < 20; i++)
    {
        sum = sum + a[i] * b[i];
    }
    printf("Sum: %d ", sum);
    printf("\n");

    return 0;
}