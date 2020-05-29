#include "execute.h"

void main(){
    
    if(!access("info.exe",0)) system("del info.exe"); 
    if(!access("info.dat",0)) system("del info.dat"); 
    if(!access("info.txt",0)) system("del info.txt");  
    if(!access("title.txt",0)) system("del title.txt"); 

    system("gcc -o info main.c controller.c observer.c");
    system("info.exe"); 
    system("python BSRMPlot.py"); 
}