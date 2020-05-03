#ifndef ACMSIM_H  //avoid repeating define 
#define ACMSIM_H  //define the .h file name



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

// SYSTEM FRE SETTING FOR MEASURE, CTRL, SIMULATION 
#define NUMBER_OF_STEPS 2000000  //total step for 1 second
#define NS NUMBER_OF_STEPS 
#define TIME_EACH_STEP 0.0000005  //1s for 5000000 steps
#define WRITE_FRE 50000
#define WRITE_PER_TIME 0.00002
#define CTRL_FRE 10000
#define MEASURE_FRE 10000




#define TRUE  True
#define FALSE False
#define True  true
#define False false
#define true  (1)
#define false (0)
#define pi 3.141592653589793238 /* double */
#define u0 0.0000012566
#define ONE_OVER_2PI          0.159154943 // 1/(2*pi)
#define TWO_PI_OVER_3         2.0943951 // 2*pi/3
#define SIN_2PI_SLASH_3       0.8660254 // sin(2*pi/3)
#define SIN_DASH_2PI_SLASH_3 -0.8660254 // sin(-2*pi/3)
#define SQRT_2_SLASH_3        0.81649658 // sqrt(2.0/3.0)
#define fbs fabs  //  use_fabs_instead_or_you_will_regret
#define PI_OVER_180   0.0174533
#define sqrt3 1.732


//the define for DSP expriment
#define Uint32 unsigned long int
#define Uint16 unsigned int


//MOTOR IMFORMATION
#define NUMBER_OF_STATORS 12 // valid for BSRM
#define NUMBER_OF_ROTORS 8 // valid for BSRM
#define PHASE_NUMBER 3 // 3 phase machine
#define H_STACK 0.05
#define POLE_ARC pi/12
#define RADIUS 0.02478
#define l_0 0.00022
#define AIRGAP_LENGTH  l_0
#define COMMUTATE_ANGLE pi/12
#define friction_factor 0.005//for the rpm speed
#define c 1.49
#define Nm 14/3
#define Ns 11/2
#define J 2.06e-6


// Everthing else is in here



//STRUCTURE DEFINE to descripe the state of motor
struct BSRMSimulated{

    double TIME;

//TEH STATE OF ROTOR MECHANIC PART 
    double mech_angle;
    double angle_A;
    double angle_B;
    double angle_C;
    
    double m;
    double rpm;
    double mech_w;
    double mech_w_deriv;//be converted to angle accelerating speed

    double Tload;// load torque
    double X_load;// load torque
    double Y_load;// load torque
    double Tem_A;//ele torque
    double Tem_B;
    double Tem_C;
    double Tem;

    double x_displacement;
    double y_displacement;
    double x_v;
    double y_v;
 
    double Kf1_A;
    double Kf2_A;
    double Kf1_B;
    double Kf2_B;
    double Kf1_C;
    double Kf2_C;

    double Rm;
    double Um; 
    double Rs; 
    double Us; 

    double x_force_A;
    double y_force_A;
    double x_force_B;
    double y_force_B;
    double x_force_C;
    double y_force_C;
    double x_force;
    double y_force;
    


// THE ELECTRICAL PART
// INDUCTANCE IS CAHNGING
    double LA_facotr;
    double LB_facotr;
    double LC_facotr;  
    double LA_deri_factor;
    double LB_deri_factor;
    double LC_deri_factor; 


//WINDINGS STATE
    double IA;
    double IB;
    double IC;
    double IA_X;
    double IB_X;
    double IC_X;
    double IA_Y;
    double IB_Y;
    double IC_Y;
};
extern struct BSRMSimulated BSRM;



// Saturation
// #include "satlut.h"
// #define ACMSIMC_DEBUG false

/* Declaration of Utility Function */
void write_header_to_file(FILE *fw);
void write_data_to_file(FILE *fw);
int isNumber(double x);
double sign(double x);
double fabs(double x);

#endif

