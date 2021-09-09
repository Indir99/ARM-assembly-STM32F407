#include<stdio.h>
#include<string.h>


int main(){
    char *poruka="KAKO STE DANAS";
    int i=0;
    while(poruka[i]!='\0'){
        i++;
    }
    printf("Vrijednost brojaca je: %d ",i);
    printf("\n");

    return 0;
}