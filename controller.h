#ifndef CONTROLLER_H
#define CONTROLLER_H

#define SPEED_LOOP_PID_PROPORTIONAL_GAIN 0.001
#define SPEED_LOOP_PID_INTEGRAL 0.000015
#define SPEED_LOOP_PID_DIREVATIVE 0
#define SPEED_LOOP_LIMIT_ANGLE pi/36  //5° instead 7.5°

#define DIS_LOOP_PID_PROPORTIONAL_GAIN 50000
#define DIS_LOOP_PID_INTEGRAL 0
#define DIS_LOOP_PID_DIREVATIVE 200
#define DIS_LOOP_LIMIT 10

#define CURRENT_HYSTERSIS 0.1
#define CURRENT_MIAN_WINDINGS 10


struct PID_Reg{
   double Kp; // Proportional gain
   double Ki; // Integral 
   double Kd; // Differential 
   double limit; // Output limit
   double pre_error;
   double prepre_error;
   double pre_output;
   double output_incre;
   double output;
   double cutoff;
};


double PID(struct PID_Reg *r, double error);


struct ControllerForExperiment{

//TEH STATE OF ROTOR MECHANIC PART 
    double mech_angle;
    double angle_A;
    double angle_B;
    double angle_C;
    double advance_angle;
    double main_current;
    double current_hystersis;
    
    int ctrl_count;
    int measure_count;

    double m;
    double rpm;
    double rpm_cmd;
    double mech_w;
    double mech_w_deriv;//be converted to angle accelerating speed



    double x_displacement;
    double y_displacement;

 
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
    double IA_X;
    double IB_X;
    double IC_X;
    double IA_Y;
    double IB_Y;
    double IC_Y;
    double UA;
    double UB;
    double UC;
    double UA_X;
    double UB_X;
    double UC_X;
    double UA_Y;
    double UB_Y;
    double UC_Y;
    
    struct PID_Reg PID_speed;
    struct PID_Reg PID_disx;//displacement
    struct PID_Reg PID_disy;
};
extern struct ControllerForExperiment CTRL;

void CTRL_init();

void control(int step);


void circuit_main_A();
void circuit_main_B();
void circuit_main_C();

#endif
