#include "modify.h"

void main(){
    
    FILE *fp_read = NULL;
    FILE *fp_write = NULL;
    char buff[255]="begin";
    char temp_buff_for_print[255];
    float sim_fre=2000000;
    char need_to_be_judge[50]="#define NUMBER_OF_STEPS";
    char end[10]="endup";
    char* exist_judge;

    printf("%s",end);
    fp_read = fopen("BSRMSim_temp.h", "r+");
    fp_write = fopen("BSRMSim.h", "w+");   


    while ( true)   //if str1= str2 ，then strcmp(str1, str2)=0
    {
        printf("%s",temp_buff_for_print);
        fgets(buff, 255, (FILE*)fp_read);
        //the fget will read one row if executed once and the pointer fp will pointer to next row
        strncpy(temp_buff_for_print, buff, 255);
        // copy the first 10 char of buff to temp_buff_for_print 
        if( strstr(temp_buff_for_print, need_to_be_judge) ){
            fprintf(fp_write, "%s %f\n", need_to_be_judge ,sim_fre);
        }
        else{
            fprintf(fp_write, "%s",temp_buff_for_print);
        }
        if ( strstr(temp_buff_for_print,end) ){break;}

    }
    fclose(fp_read);       
    fclose(fp_write);
//    getch();
//    fprintf(fp, "//This is testing 1 for fprintf...\n");   //write a row, the \n is necessary if want to read later
/*
    strncpy(temp_buff_for_print, buff, 255);                 // copy the first 10 char of buff to temp_buff_for_print 
    exist_judge=strstr(temp_buff_for_print, need_to_be_judge);    // return the address of the first pairing char
    if(NULL == exist_judge)
        printf("no exist");
    else
        printf("%s\n", temp_buff_for_print);

    fgets(buff, 255, (FILE*)fp);          //buff is update every time
    strncpy(temp_buff_for_print, buff, 255);   // copy the first 10 char of buff to temp_buff_for_print 
    exist_judge=strstr(temp_buff_for_print, need_to_be_judge);    // return the address of the first pairing chars
    if(NULL == exist_judge)
        printf("no exist");
    else
        printf("%s\n", temp_buff_for_print);
 */       
}

//  gcc modify.c -o modify

