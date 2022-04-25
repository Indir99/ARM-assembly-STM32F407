#include<stdio.h>


int main(){
    int x,y;
    printf("Unesite vrijednost za varijablu x: ");
    scanf("%d",&x);
    if(x>10){
        y=11;
    }
    else if(x<10){
        y=9;
    }
    else{
        y=10;
    }
    printf("Vrijednost varijable y je: %d",y);
    printf("\n");
    

    return 0;
}