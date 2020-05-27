/* standard lib */
// #include <stdbool.h> // bool for _Bool and true for 1
#include <stdio.h> // printf #include <stdbool.h> // bool for _Bool and true for 1
#include <process.h>//reqd. for system function prototype  
// like system("python ./Plot.py");
#include <conio.h> //this function is used to control the program by console
// for clrscr clear screen .getch() get the input char
#include "stdlib.h" // for rand()
#include "math.h" //special funciton like sin() sqrt() abs()
#include "time.h"//count the running time
#include "controller.h"
#include "BSRMSim.h"
#include "observer.c"
#include "observer.h"

int main(){
    
    system("gcc -o info main.c controller.c observer.c");
    system("info.exe"); 
    system("python BSRMPlot.py"); 
}


