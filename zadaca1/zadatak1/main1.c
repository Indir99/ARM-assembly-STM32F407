#include<stdio.h>

//Prvi zadatak implementiran u C-u


int main(){
    int a[20];
    int b[20];
    int s=1;
    for(int i=0;i<20;i++){
        a[i]=s;
        s++;
    }
    s=1;
    for(int i=0;i<20;i++){
        b[i]=s;
        s++;
    }
    printf("Elementi niza a su:");
    for(int i=0;i<20;i++){
        printf(" %d", a[i]);
    }
    printf("\n");
    printf("Elementi niza b su:");
    for(int i=0;i<20;i++){
        printf(" %d", b[i]);
    }
    printf("\n");
    int suma=0;
    for(int i=0;i<20;i++){
        suma=suma + a[i]*b[i];
    }
    printf("Suma je: %d ",suma);
    printf("\n");


   return 0; 
}