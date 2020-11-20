#include "BSRMSim.h"


struct BSRMSimulated BSRM;
struct ControllerForExperiment CTRL;


void write_header_to_file(FILE *fw){// include the explaination of data imformation
    // no space is allowed!
    //BSRM.IA(Current)[A],BSRM.IA_X(Current)[A],BSRM.IA_Y(Current)[A],BSRM.IB(Current)[A],BSRM.IC(Current)[A],BSRM.Kf1_A(Factor),BSRM.Kf2_A(Factor),BSRM.LA(Inductance)[H],BSRM.mech_angle(position)[rad],BSRM.rpm(Speed)[rpm],BSRM.rpm_cmd(Speed)[rpm],BSRM.Tem(Torque)[N*m],CTRL.UA(Voltage)[V],CTRL.UA_X(Voltage)[V],CTRL.UA_Y(Voltage)[V],CTRL.UB(Voltage)[V],CTRL.UC(Voltage)[V],BSRM.Tem,BSRM.Tem_A,BSRM.Tem_B,BSRM.Tem_C,BSRM.Tload,BSRM.mech_angle,BSRM.timebase(Time)[s],CTRL.advance_angle(Angle)[rad],CTRL.angle_A(Angle)[rad],CTRL.mech_angle(Angle)[rad],CTRL.rpm(Speed)[rpm],CTRL.UA\n"
    fprintf(fw, "Time/(s),Velocity/(rpm),Current/(A),Current/(A),Current/(A),Angle/(rad),Angle/(rad),Tem/(Nm),Current/(A),Current/(A),Current/(A),Current/(A),Force/(N),Force/(N),Force/(N),Force/(N),Displacement/(um),Displacement/(um),Velocity/(m/s),Velocity/(m/s),Angle/(rad)\n");    
    {
        FILE *fw2;
        fw2 = fopen("info.dat", "w");   //create a new file named info.dat. If exist, clear all.
        fprintf(fw2, "SIMULATION_FRE,WRITE_PER_TIME,DATA_FILE_NAME\n");
        fprintf(fw2, "%d, %f, %s\n", NS, WRITE_PER_TIME , "info.txt");
        //%f   output as format float 3.1415926
        //%e   output like 3.141593e+000
        //%g   automatically choose output format 3.14159
        //%d   output like 3
        //%s   string
        fclose(fw2);
    }
    {
        FILE *fw3;
        fw3 = fopen("title.txt", "w");   //create a new file named title.txt. If exist, clear all.
        fprintf(fw3, "Time,Rotational Speed,Current in A Phase,Current in B Phase,Current in C Phase,Angle of A Phase,Angle of Rotor,Total Torque,Current of A Phase Suspension Winding in X Axis,Current of A Phase Suspension Winding in Y Axis,Current Order of A Phase Suspension Winding in X Axis,Current Order of A Phase Suspension Winding in Y Axis,Order of Force in X Axis,Order of Force in Y Axis,Force in X Axis,Force in Y Axis,Displacement in X Axis,Displacement in Y Axis,Velocity in X Axis,Velocity in Y Axis,Advance Angle\n");
        fclose(fw3);
    }

}

void write_data_to_file(FILE *fw){

    static int write_judge=0; // write_judge for write sampling

    {   
        write_judge++;
        if(write_judge == NUMBER_OF_STEPS/WRITE_FRE )
        {
            write_judge=0;
            // no space is allowed!!!23
            fprintf(fw, "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n",BSRM.TIME,BSRM.rpm,BSRM.IA,BSRM.IB,BSRM.IC,BSRM.angle_A,BSRM.mech_angle,BSRM.Tem,BSRM.IA_X,BSRM.IA_Y,CTRL.IA_X,CTRL.IA_Y,CTRL.x_force,CTRL.y_force,BSRM.x_force,BSRM.y_force,BSRM.x_displacement*1000000,BSRM.y_displacement*1000000,BSRM.x_v,BSRM.y_v,CTRL.advance_angle //20
                    );
        }
        
    }

}

int isNumber(double x){
    // This looks like it should always be true, 
    // but it's false if x is an NaN (1.#QNAN0).
    return (x == x); 
    // see https://www.johndcook.com/blog/IEEE_exceptions_in_cpp/ cb: https://stackoverflow.com/questions/347920/what-do-1-inf00-1-ind00-and-1-ind-mean
}

double sign(double x){
    return (x > 0) - (x < 0);    
}


void angle_trans(double mech_angle){
    if(fmod(mech_angle,pi/4) <= pi/8) {BSRM.angle_A=fmod(mech_angle,pi/4);}
    else {BSRM.angle_A=fmod(mech_angle,pi/4)-pi/4;}//A phase
    if(fmod(mech_angle-pi/12,pi/4) <= pi/8) {BSRM.angle_B=fmod(mech_angle-pi/12,pi/4);}
    else {BSRM.angle_B=fmod(mech_angle-pi/12,pi/4)-pi/4;}
    if(fmod(mech_angle+pi/12,pi/4) <= pi/8) {BSRM.angle_C=fmod(mech_angle+pi/12,pi/4);}
    else {BSRM.angle_C=fmod(mech_angle+pi/12,pi/4)-pi/4;}
}
void cal_Kf_A(double angle){
    BSRM.Kf1_A=Nm*Ns*(u0*H_STACK*RADIUS*(pi-12*angle)/6/(l_0*l_0)+4*u0*H_STACK*RADIUS*(2*RADIUS*angle+l_0)*angle/(pi*RADIUS*RADIUS*angle*angle+(pi+2)*RADIUS*angle*l_0+2*l_0*l_0)/l_0);
    BSRM.Kf2_A=Nm*Ns*(u0*H_STACK*RADIUS*(pi-12*angle)*angle/12/(l_0*l_0)-2*u0*H_STACK/l_0+4*u0*H_STACK*RADIUS*(2*RADIUS*angle+l_0)*angle/(pi*RADIUS*RADIUS*angle*angle+(pi+2)*RADIUS*angle*l_0+2*l_0*l_0));
    //h_stack,r,l_air,Nm,Ns
}
void cal_Kf_B(double angle){
    BSRM.Kf1_B=Nm*Ns*(u0*H_STACK*RADIUS*(pi-12*angle)/6/(l_0*l_0)+4*u0*H_STACK*RADIUS*(2*RADIUS*angle+l_0)*angle/(pi*RADIUS*RADIUS*angle*angle+(pi+2)*RADIUS*angle*l_0+2*l_0*l_0)/l_0);
    BSRM.Kf2_B=Nm*Ns*(u0*H_STACK*RADIUS*(pi-12*angle)*angle/12/(l_0*l_0)-2*u0*H_STACK/l_0+4*u0*H_STACK*RADIUS*(2*RADIUS*angle+l_0)*angle/(pi*RADIUS*RADIUS*angle*angle+(pi+2)*RADIUS*angle*l_0+2*l_0*l_0));
    //h_stack,r,l_air,Nm,Ns
}
void cal_Kf_C(double angle){
    BSRM.Kf1_C=Nm*Ns*(u0*H_STACK*RADIUS*(pi-12*angle)/6/(l_0*l_0)+4*u0*H_STACK*RADIUS*(2*RADIUS*angle+l_0)*angle/(pi*RADIUS*RADIUS*angle*angle+(pi+2)*RADIUS*angle*l_0+2*l_0*l_0)/l_0);
    BSRM.Kf2_C=Nm*Ns*(u0*H_STACK*RADIUS*(pi-12*angle)*angle/12/(l_0*l_0)-2*u0*H_STACK/l_0+4*u0*H_STACK*RADIUS*(2*RADIUS*angle+l_0)*angle/(pi*RADIUS*RADIUS*angle*angle+(pi+2)*RADIUS*angle*l_0+2*l_0*l_0));
    //h_stack,r,l_air,Nm,Ns
} 

void cal_force_A(double angle_A,double IA,double IA_X,double IA_Y){
    if (fbs(IA)>0.1){
        cal_Kf_A(angle_A);
        BSRM.x_force_A=IA*(BSRM.Kf1_A*IA_X+BSRM.Kf2_A*IA_Y);
        BSRM.y_force_A=IA*(-BSRM.Kf2_A*IA_X+BSRM.Kf1_A*IA_Y);
    }
    else{
        BSRM.x_force_A=0;
        BSRM.y_force_A=0;
    }
}
void cal_force_B(double angle_B,double IB,double IB_X,double IB_Y){
    if (fbs(IB)>0.1){
        cal_Kf_B(angle_B);
        BSRM.x_force_B=IB*(BSRM.Kf1_B*IB_X+BSRM.Kf2_B*IB_Y);
        BSRM.y_force_B=IB*(-BSRM.Kf2_B*IB_X+BSRM.Kf1_B*IB_Y);
    }
    else{
        BSRM.x_force_B=0;
        BSRM.y_force_B=0;
    }
}
void cal_force_C(double angle_C,double IC,double IC_X,double IC_Y){
    if (fbs(IC)>0.1){
        cal_Kf_C(angle_C);
        BSRM.x_force_C=IC*(BSRM.Kf1_C*IC_X+BSRM.Kf2_C*IC_Y);
        BSRM.y_force_C=IC*(-BSRM.Kf2_C*IC_X+BSRM.Kf1_C*IC_Y);
    }
    else{
        BSRM.x_force_C=0;
        BSRM.y_force_C=0;
    }
}   
 
void cal_torque_A(double angle_A,double IA,double IA_X,double IA_Y){
    if (IA>0.1){
        if(angle_A<0){
            BSRM.Tem_A=(Nm*Nm*IA*IA+Ns*Ns*IA_X*IA_X/2+Ns*Ns*IA_Y*IA_Y/2)*(2*u0*RADIUS*H_STACK/l_0+4*u0*H_STACK*RADIUS*(2*RADIUS*angle_A-l_0)/(pi*RADIUS*RADIUS*angle_A*angle_A-(pi+2)*RADIUS*angle_A*l_0+2*l_0*l_0));
            }
        else {
            BSRM.Tem_A=(Nm*Nm*IA*IA+Ns*Ns*IA_X*IA_X/2+Ns*Ns*IA_Y*IA_Y/2)*(-2*u0*RADIUS*H_STACK/l_0+4*u0*H_STACK*RADIUS*(2*RADIUS*angle_A+l_0)/(pi*RADIUS*RADIUS*angle_A*angle_A+(pi+2)*RADIUS*angle_A*l_0+2*l_0*l_0));
            } 
    }  
    else  
        BSRM.Tem_A=0;       
}
void cal_torque_B(double angle_B,double IB,double IB_X,double IB_Y){
    if (IB>0.1){
       if(angle_B<0){
            BSRM.Tem_B=(Nm*Nm*IB*IB+Ns*Ns*IB_X*IB_X/2+Ns*Ns*IB_Y*IB_Y/2)*(2*u0*RADIUS*H_STACK/l_0+4*u0*H_STACK*RADIUS*(2*RADIUS*angle_B-l_0)/(pi*RADIUS*RADIUS*angle_B*angle_B-(pi+2)*RADIUS*angle_B*l_0+2*l_0*l_0));
            }
        else {
            BSRM.Tem_B=(Nm*Nm*IB*IB+Ns*Ns*IB_X*IB_X/2+Ns*Ns*IB_Y*IB_Y/2)*(-2*u0*RADIUS*H_STACK/l_0+4*u0*H_STACK*RADIUS*(2*RADIUS*angle_B+l_0)/(pi*RADIUS*RADIUS*angle_B*angle_B+(pi+2)*RADIUS*angle_B*l_0+2*l_0*l_0));
            }     }
    else {
        BSRM.Tem_B=0;
    }
}
void cal_torque_C(double angle_C,double IC,double IC_X,double IC_Y){
    if (IC>0.1){
        if(angle_C<0){
            BSRM.Tem_C=(Nm*Nm*IC*IC+Ns*Ns*IC_X*IC_X/2+Ns*Ns*IC_Y*IC_Y/2)*(2*u0*RADIUS*H_STACK/l_0+4*u0*H_STACK*RADIUS*(2*RADIUS*angle_C-l_0)/(pi*RADIUS*RADIUS*angle_C*angle_C-(pi+2)*RADIUS*angle_C*l_0+2*l_0*l_0));
            }
        else {
            BSRM.Tem_C=(Nm*Nm*IC*IC+Ns*Ns*IC_X*IC_X/2+Ns*Ns*IC_Y*IC_Y/2)*(-2*u0*RADIUS*H_STACK/l_0+4*u0*H_STACK*RADIUS*(2*RADIUS*angle_C+l_0)/(pi*RADIUS*RADIUS*angle_C*angle_C+(pi+2)*RADIUS*angle_C*l_0+2*l_0*l_0));
            } 
    }
    else {
        BSRM.Tem_C=0;
    }
}

void cal_L_factor_A(double angle){
    BSRM.LA_facotr=u0*H_STACK*RADIUS*(pi-12*angle)/6/l_0+4*u0*H_STACK/pi*log((pi*RADIUS*RADIUS*angle*angle+(pi+2)*RADIUS*angle*l_0+2*l_0*l_0)/(2*l_0*l_0))-8*u0*H_STACK/pi/(pi-2)*log((pi*RADIUS*RADIUS*angle*angle+(pi+2)*RADIUS*angle*l_0+2*l_0*l_0)/2/(RADIUS*angle+l_0)/(RADIUS*angle+l_0));  
}
void cal_L_factor_B(double angle){   
    BSRM.LB_facotr=u0*H_STACK*RADIUS*(pi-12*angle)/6/l_0+4*u0*H_STACK/pi*log((pi*RADIUS*RADIUS*angle*angle+(pi+2)*RADIUS*angle*l_0+2*l_0*l_0)/(2*l_0*l_0))-8*u0*H_STACK/pi/(pi-2)*log((pi*RADIUS*RADIUS*angle*angle+(pi+2)*RADIUS*angle*l_0+2*l_0*l_0)/2/(RADIUS*angle+l_0)/(RADIUS*angle+l_0)); 
}
void cal_L_factor_C(double angle){   
    BSRM.LC_facotr=u0*H_STACK*RADIUS*(pi-12*angle)/6/l_0+4*u0*H_STACK/pi*log((pi*RADIUS*RADIUS*angle*angle+(pi+2)*RADIUS*angle*l_0+2*l_0*l_0)/(2*l_0*l_0))-8*u0*H_STACK/pi/(pi-2)*log((pi*RADIUS*RADIUS*angle*angle+(pi+2)*RADIUS*angle*l_0+2*l_0*l_0)/2/(RADIUS*angle+l_0)/(RADIUS*angle+l_0)); 
}


void cal_L_deri_factor_A(double angle_below_zero,double angle){   
    BSRM.LA_deri_factor=2*u0*H_STACK*RADIUS/l_0+(4*u0*H_STACK/pi-8*u0*H_STACK/pi/(pi-2))*(2*pi*RADIUS*RADIUS*angle_below_zero-RADIUS*l_0*(pi+2))/(pi*RADIUS*RADIUS*angle_below_zero*angle_below_zero-(pi+2)*RADIUS*angle_below_zero*l_0+2*l_0*l_0)-16*u0*H_STACK/pi/(pi-2)*RADIUS/(l_0-RADIUS*angle_below_zero);
    if(angle>=0){BSRM.LA_deri_factor=-BSRM.LA_deri_factor;}
}
void cal_L_deri_factor_B(double angle_below_zero,double angle){
    BSRM.LB_deri_factor=2*u0*H_STACK*RADIUS/l_0+(4*u0*H_STACK/pi-8*u0*H_STACK/pi/(pi-2))*(2*pi*RADIUS*RADIUS*angle_below_zero-RADIUS*l_0*(pi+2))/(pi*RADIUS*RADIUS*angle_below_zero*angle_below_zero-(pi+2)*RADIUS*angle_below_zero*l_0+2*l_0*l_0)-16*u0*H_STACK/pi/(pi-2)*RADIUS/(l_0-RADIUS*angle_below_zero);
    if(angle>=0){BSRM.LB_deri_factor=-BSRM.LB_deri_factor;}
}
void cal_L_deri_factor_C(double angle_below_zero,double angle){
    BSRM.LC_deri_factor=2*u0*H_STACK*RADIUS/l_0+(4*u0*H_STACK/pi-8*u0*H_STACK/pi/(pi-2))*(2*pi*RADIUS*RADIUS*angle_below_zero-RADIUS*l_0*(pi+2))/(pi*RADIUS*RADIUS*angle_below_zero*angle_below_zero-(pi+2)*RADIUS*angle_below_zero*l_0+2*l_0*l_0)-16*u0*H_STACK/pi/(pi-2)*RADIUS/(l_0-RADIUS*angle_below_zero);
    if(angle>=0){BSRM.LC_deri_factor=-BSRM.LC_deri_factor;}
}


/* Simple Model based on the differential equaiton */
void dynamics(){
    /* raduis force in both axis and Tem */
 
    angle_trans(BSRM.mech_angle);

    cal_force_A(fbs(BSRM.angle_A),BSRM.IA,BSRM.IA_X,BSRM.IA_Y);
    cal_force_B(fbs(BSRM.angle_B),BSRM.IB,BSRM.IB_X,BSRM.IB_Y);
    cal_force_C(fbs(BSRM.angle_C),BSRM.IC,BSRM.IC_X,BSRM.IC_Y);

    cal_torque_A(BSRM.angle_A,BSRM.IA,BSRM.IA_X,BSRM.IA_Y);// UTILIZE TWO DIFFERENT EQUATIONS TO CALCULATE THE TORQUE IN BOTH RANGE
    cal_torque_B(BSRM.angle_B,BSRM.IB,BSRM.IB_X,BSRM.IB_Y);
    cal_torque_C(BSRM.angle_C,BSRM.IC,BSRM.IC_X,BSRM.IC_Y);

    cal_L_factor_A(fbs(BSRM.angle_A));
    cal_L_factor_B(fbs(BSRM.angle_B));
    cal_L_factor_C(fbs(BSRM.angle_C));

    cal_L_deri_factor_A(-fbs(BSRM.angle_A),BSRM.angle_A);
    cal_L_deri_factor_B(-fbs(BSRM.angle_B),BSRM.angle_B);
    cal_L_deri_factor_C(-fbs(BSRM.angle_C),BSRM.angle_C);

    BSRM.Tem=BSRM.Tem_A+BSRM.Tem_B+BSRM.Tem_C;

    // require to be modify
    BSRM.x_force= BSRM.x_force_A+(BSRM.x_force_B+BSRM.x_force_C)*sqrt3/2+(-BSRM.y_force_B+BSRM.y_force_C)*1/2+0*1300000*BSRM.x_displacement;
    BSRM.y_force= BSRM.y_force_A+(BSRM.y_force_B+BSRM.y_force_C)*sqrt3/2+(BSRM.x_force_B-BSRM.x_force_C)*1/2+0*1300000*BSRM.y_displacement;

    
}

void Range_Kuta(int step){
    
    double w_k1, w_k2, w_k3, w_k4, w_k;
    double x_k1, x_k2, x_k3, x_k4, x_k;
    double y_k1, y_k2, y_k3, y_k4, y_k;
    double theta_k1, theta_k2, theta_k3, theta_k4, theta_k;
    double x_v_k1, x_v_k2, x_v_k3, x_v_k4, x_v_k;
    double y_v_k1, y_v_k2, y_v_k3, y_v_k4, y_v_k;
    double imA_k1, imA_k2, imA_k3, imA_k4, imA_k;
    double ixA_k1, ixA_k2, ixA_k3, ixA_k4, ixA_k;
    double iyA_k1, iyA_k2, iyA_k3, iyA_k4, iyA_k;
    double imB_k1, imB_k2, imB_k3, imB_k4, imB_k;
    double ixB_k1, ixB_k2, ixB_k3, ixB_k4, ixB_k;
    double iyB_k1, iyB_k2, iyB_k3, iyB_k4, iyB_k;
    double imC_k1, imC_k2, imC_k3, imC_k4, imC_k;
    double ixC_k1, ixC_k2, ixC_k3, ixC_k4, ixC_k;
    double iyC_k1, iyC_k2, iyC_k3, iyC_k4, iyC_k;

    //store the value for runge kuta calculation
    double f_w,f_angle,fx_v,fy_v,fx_dis,fy_dis,fma,f1a,f2a,fmb,f1b,f2b,fmc,f1c,f2c;
    
//    double f_LA_deri_factor,f_LA_facotr,f_LB_deri_factor,f_LB_facotr,f_LC_deri_factor,f_LC_facotr;
    dynamics(); // timer.t,
    
    //the initial state should be preserved for last step of  Runge-Kutta simulation
    f_angle=BSRM.mech_angle;
    f_w=BSRM.mech_w;
    fx_v=BSRM.x_v;
    fy_v=BSRM.y_v;
    fx_dis=BSRM.x_displacement;
    fy_dis=BSRM.y_displacement;
    fma=BSRM.IA;
    f1a=BSRM.IA_X;
    f2a=BSRM.IA_Y;
    fmb=BSRM.IB;
    f1b=BSRM.IB_X;
    f2b=BSRM.IB_Y;
    fmc=BSRM.IC;
    f1c=BSRM.IC_X;
    f2c=BSRM.IC_Y; 
/*
    f_LA_deri_factor=BSRM.LA_deri_factor;
    f_LA_facotr=BSRM.LA_facotr;
    f_LB_deri_factor=BSRM.LB_deri_factor;
    f_LB_facotr=BSRM.LB_deri_factor;
    f_LC_deri_factor=BSRM.LB_deri_factor;
    f_LC_facotr=BSRM.LB_deri_factor;
*/
    //MECHNICAL PART DERITIVE
    w_k1 = (BSRM.Tem-BSRM.Tload)/J;
    theta_k1 = BSRM.mech_w;
    x_k1 = BSRM.x_v;
    y_k1 = BSRM.y_v;
    x_v_k1 = (BSRM.x_force-BSRM.X_load)/BSRM.m;
    y_v_k1 = (BSRM.y_force-BSRM.Y_load)/BSRM.m;


    //ELECTRICAL PART DERITIVE
    imA_k1=(CTRL.UA-BSRM.Rm*BSRM.IA-BSRM.mech_w*BSRM.LA_deri_factor*2*Nm*Nm*BSRM.IA)/(BSRM.LA_facotr*2*Nm*Nm);
    ixA_k1=(CTRL.UA_X-BSRM.Rs*BSRM.IA_X-BSRM.mech_w*BSRM.LA_deri_factor*Ns*Ns*BSRM.IA_X)/(BSRM.LA_facotr*Ns*Ns);
    iyA_k1=(CTRL.UA_Y-BSRM.Rs*BSRM.IA_Y-BSRM.mech_w*BSRM.LA_deri_factor*Ns*Ns*BSRM.IA_Y)/(BSRM.LA_facotr*Ns*Ns);

    imB_k1=(CTRL.UB-BSRM.Rm*BSRM.IB-BSRM.mech_w*BSRM.LB_deri_factor*2*Nm*Nm*BSRM.IB)/(BSRM.LB_facotr*2*Nm*Nm);
    ixB_k1=(CTRL.UB_X-BSRM.Rs*BSRM.IB_X-BSRM.mech_w*BSRM.LB_deri_factor*Ns*Ns*BSRM.IB_X)/(BSRM.LB_facotr*Ns*Ns);
    iyB_k1=(CTRL.UB_Y-BSRM.Rs*BSRM.IB_Y-BSRM.mech_w*BSRM.LB_deri_factor*Ns*Ns*BSRM.IB_Y)/(BSRM.LB_facotr*Ns*Ns);

    imC_k1=(CTRL.UC-BSRM.Rm*BSRM.IC-BSRM.mech_w*BSRM.LC_deri_factor*2*Nm*Nm*BSRM.IC)/(BSRM.LC_facotr*2*Nm*Nm);
    ixC_k1=(CTRL.UC_X-BSRM.Rs*BSRM.IC_X-BSRM.mech_w*BSRM.LC_deri_factor*Ns*Ns*BSRM.IC_X)/(BSRM.LC_facotr*Ns*Ns);
    iyC_k1=(CTRL.UC_Y-BSRM.Rs*BSRM.IC_Y-BSRM.mech_w*BSRM.LC_deri_factor*Ns*Ns*BSRM.IC_Y)/(BSRM.LC_facotr*Ns*Ns);

    //BEGIN TO CALCULATE HTE k+1/2 step state variables
    BSRM.mech_angle=f_angle+ TIME_EACH_STEP*theta_k1/2;
    BSRM.mech_w=f_w +TIME_EACH_STEP*w_k1/2;
    BSRM.x_v=fx_v+ TIME_EACH_STEP*x_v_k1/2;
    BSRM.y_v=fy_v+ TIME_EACH_STEP*y_v_k1/2;
    BSRM.x_displacement=fx_dis+ TIME_EACH_STEP*x_k1/2;//    BSRM.x_v=BSRM.x_v+ TIME_EACH_STEP*x_k1;
    BSRM.y_displacement=fy_dis+ TIME_EACH_STEP*y_k1/2;//    BSRM.y_v=BSRM.y_v+ TIME_EACH_STEP*y_k1;

    BSRM.IA=fma+ TIME_EACH_STEP*imA_k1/2;
    BSRM.IA_X=f1a+ TIME_EACH_STEP*ixA_k1/2;
    BSRM.IA_Y=f2a+ TIME_EACH_STEP*iyA_k1/2;
    BSRM.IB=fmb+ TIME_EACH_STEP*imB_k1/2;
    BSRM.IB_X=f1b+ TIME_EACH_STEP*ixB_k1/2;
    BSRM.IB_Y=f2b+ TIME_EACH_STEP*iyB_k1/2;
    BSRM.IC=fmc+ TIME_EACH_STEP*imC_k1/2;
    BSRM.IC_X=f1c+ TIME_EACH_STEP*ixC_k1/2;
    BSRM.IC_Y=f2c+ TIME_EACH_STEP*iyC_k1/2;

    dynamics(); // timer.t+1/2,

    // SINCE THE k2 IS UTILIZED FRO k STEP, SO THERE IS NO VALUE IN RECORDING HTE DATA, JUST CALCULATE THE DERITIVE 

    //MECHNICAL PART DERITIVE
    w_k2=(BSRM.Tem-BSRM.Tload)/J;
    theta_k2=BSRM.mech_w;
    x_k2 = BSRM.x_v;
    y_k2 = BSRM.y_v;
    x_v_k2 = (BSRM.x_force-BSRM.X_load)/BSRM.m;
    y_v_k2 = (BSRM.y_force-BSRM.Y_load)/BSRM.m;

    imA_k2=(CTRL.UA-BSRM.Rm*BSRM.IA-BSRM.mech_w*BSRM.LA_deri_factor*2*Nm*Nm*BSRM.IA)/(BSRM.LA_facotr*2*Nm*Nm);
    ixA_k2=(CTRL.UA_X-BSRM.Rs*BSRM.IA_X-BSRM.mech_w*BSRM.LA_deri_factor*Ns*Ns*BSRM.IA_X)/(BSRM.LA_facotr*Ns*Ns);
    iyA_k2=(CTRL.UA_Y-BSRM.Rs*BSRM.IA_Y-BSRM.mech_w*BSRM.LA_deri_factor*Ns*Ns*BSRM.IA_Y)/(BSRM.LA_facotr*Ns*Ns);

    imB_k2=(CTRL.UB-BSRM.Rm*BSRM.IB-BSRM.mech_w*BSRM.LB_deri_factor*2*Nm*Nm*BSRM.IB)/(BSRM.LB_facotr*2*Nm*Nm);
    ixB_k2=(CTRL.UB_X-BSRM.Rs*BSRM.IB_X-BSRM.mech_w*BSRM.LB_deri_factor*Ns*Ns*BSRM.IB_X)/(BSRM.LB_facotr*Ns*Ns);
    iyB_k2=(CTRL.UB_Y-BSRM.Rs*BSRM.IB_Y-BSRM.mech_w*BSRM.LB_deri_factor*Ns*Ns*BSRM.IB_Y)/(BSRM.LB_facotr*Ns*Ns);   
   
    imC_k2=(CTRL.UC-BSRM.Rm*BSRM.IC-BSRM.mech_w*BSRM.LC_deri_factor*2*Nm*Nm*BSRM.IC)/(BSRM.LC_facotr*2*Nm*Nm);
    ixC_k2=(CTRL.UC_X-BSRM.Rs*BSRM.IC_X-BSRM.mech_w*BSRM.LC_deri_factor*Ns*Ns*BSRM.IC_X)/(BSRM.LC_facotr*Ns*Ns);
    iyC_k2=(CTRL.UC_Y-BSRM.Rs*BSRM.IC_Y-BSRM.mech_w*BSRM.LC_deri_factor*Ns*Ns*BSRM.IC_Y)/(BSRM.LC_facotr*Ns*Ns);


    //BEGIN TO CALCULATE HTE k+1/2 step state variables
    BSRM.mech_angle=f_angle+ TIME_EACH_STEP*theta_k2/2;
    BSRM.mech_w=f_w+ TIME_EACH_STEP*w_k2/2;
    BSRM.x_v=fx_v+ TIME_EACH_STEP*x_v_k2/2;
    BSRM.y_v=fy_v+ TIME_EACH_STEP*y_v_k2/2;
    BSRM.x_displacement=fx_dis+ TIME_EACH_STEP*x_k2/2;//    BSRM.x_v=BSRM.x_v+ TIME_EACH_STEP*x_k1;
    BSRM.y_displacement=fy_dis+ TIME_EACH_STEP*y_k2/2;//    BSRM.y_v=BSRM.y_v+ TIME_EACH_STEP*y_k1;

    BSRM.IA=fma+ TIME_EACH_STEP*imA_k2/2;
    BSRM.IA_X=f1a+ TIME_EACH_STEP*ixA_k2/2;
    BSRM.IA_Y=f2a+ TIME_EACH_STEP*iyA_k2/2;
    BSRM.IB=fmb+ TIME_EACH_STEP*imB_k2/2;
    BSRM.IB_X=f1b+ TIME_EACH_STEP*ixB_k2/2;
    BSRM.IB_Y=f2b+ TIME_EACH_STEP*iyB_k2/2;
    BSRM.IC=fmc+ TIME_EACH_STEP*imC_k2/2;
    BSRM.IC_X=f1c+ TIME_EACH_STEP*ixC_k2/2;
    BSRM.IC_Y=f2c+ TIME_EACH_STEP*iyC_k2/2;
    

    dynamics(); // timer.t+1/2, second timw

    // SINCE THE k3 IS UTILIZED FRO k STEP, SO THERE IS NO VALUE IN RECORDING HTE DATA, JUST CALCULATE THE DERITIVE 

    //MECHNICAL PART DERITIVE
    w_k3=(BSRM.Tem-BSRM.Tload)/J;
    theta_k3=BSRM.mech_w;
    x_k3 = BSRM.x_v;
    y_k3 = BSRM.y_v;
    x_v_k3 = (BSRM.x_force-BSRM.X_load)/BSRM.m;
    y_v_k3 = (BSRM.y_force-BSRM.Y_load)/BSRM.m;

    imA_k3=(CTRL.UA-BSRM.Rm*BSRM.IA-BSRM.mech_w*BSRM.LA_deri_factor*2*Nm*Nm*BSRM.IA)/(BSRM.LA_facotr*2*Nm*Nm);
    ixA_k3=(CTRL.UA_X-BSRM.Rs*BSRM.IA_X-BSRM.mech_w*BSRM.LA_deri_factor*Ns*Ns*BSRM.IA_X)/(BSRM.LA_facotr*Ns*Ns);
    iyA_k3=(CTRL.UA_Y-BSRM.Rs*BSRM.IA_Y-BSRM.mech_w*BSRM.LA_deri_factor*Ns*Ns*BSRM.IA_Y)/(BSRM.LA_facotr*Ns*Ns);

    imB_k3=(CTRL.UB-BSRM.Rm*BSRM.IB-BSRM.mech_w*BSRM.LB_deri_factor*2*Nm*Nm*BSRM.IB)/(BSRM.LB_facotr*2*Nm*Nm);
    ixB_k3=(CTRL.UB_X-BSRM.Rs*BSRM.IB_X-BSRM.mech_w*BSRM.LB_deri_factor*Ns*Ns*BSRM.IB_X)/(BSRM.LB_facotr*Ns*Ns);
    iyB_k3=(CTRL.UB_Y-BSRM.Rs*BSRM.IB_Y-BSRM.mech_w*BSRM.LB_deri_factor*Ns*Ns*BSRM.IB_Y)/(BSRM.LB_facotr*Ns*Ns);   
   
    imC_k3=(CTRL.UC-BSRM.Rm*BSRM.IC-BSRM.mech_w*BSRM.LC_deri_factor*2*Nm*Nm*BSRM.IC)/(BSRM.LC_facotr*2*Nm*Nm);
    ixC_k3=(CTRL.UC_X-BSRM.Rs*BSRM.IC_X-BSRM.mech_w*BSRM.LC_deri_factor*Ns*Ns*BSRM.IC_X)/(BSRM.LC_facotr*Ns*Ns);
    iyC_k3=(CTRL.UC_Y-BSRM.Rs*BSRM.IC_Y-BSRM.mech_w*BSRM.LC_deri_factor*Ns*Ns*BSRM.IC_Y)/(BSRM.LC_facotr*Ns*Ns);

    //BEGIN TO CALCULATE HTE k+1 step state variables
    BSRM.mech_angle=f_angle+ TIME_EACH_STEP*theta_k3;
    BSRM.mech_w=f_w+ TIME_EACH_STEP*w_k3;
    BSRM.x_v=fx_v+ TIME_EACH_STEP*x_v_k3;
    BSRM.y_v=fy_v+ TIME_EACH_STEP*y_v_k3;
    BSRM.x_displacement=fx_dis+ TIME_EACH_STEP*x_k3;//    BSRM.x_v=BSRM.x_v+ TIME_EACH_STEP*x_k1;
    BSRM.y_displacement=fy_dis+ TIME_EACH_STEP*y_k3;//    BSRM.y_v=BSRM.y_v+ TIME_EACH_STEP*y_k1;

    BSRM.IA=fma+ TIME_EACH_STEP*imA_k3;
    BSRM.IA_X=f1a+ TIME_EACH_STEP*ixA_k3;
    BSRM.IA_Y=f2a+ TIME_EACH_STEP*iyA_k3;
    BSRM.IB=fmb+ TIME_EACH_STEP*imB_k3;
    BSRM.IB_X=f1b+ TIME_EACH_STEP*ixB_k3;
    BSRM.IB_Y=f2b+ TIME_EACH_STEP*iyB_k3;
    BSRM.IC=fmc+ TIME_EACH_STEP*imC_k3;
    BSRM.IC_X=f1c+ TIME_EACH_STEP*ixC_k3;
    BSRM.IC_Y=f2c+ TIME_EACH_STEP*iyC_k3;
    
    
    dynamics(); // timer.t+1 

    // SINCE THE k4 IS UTILIZED FRO k STEP, SO THERE IS NO VALUE IN RECORDING HTE DATA, JUST CALCULATE THE DERITIVE 

    //MECHNICAL PART DERITIVE
    w_k4=(BSRM.Tem-BSRM.Tload)/J;
    theta_k4=BSRM.mech_w;
    x_k4 = BSRM.x_v;
    y_k4 = BSRM.y_v;
    x_v_k4 = (BSRM.x_force-BSRM.X_load)/BSRM.m;
    y_v_k4 = (BSRM.y_force-BSRM.Y_load)/BSRM.m;

    imA_k4=(CTRL.UA-BSRM.Rm*BSRM.IA-BSRM.mech_w*BSRM.LA_deri_factor*2*Nm*Nm*BSRM.IA)/(BSRM.LA_facotr*2*Nm*Nm);
    ixA_k4=(CTRL.UA_X-BSRM.Rs*BSRM.IA_X-BSRM.mech_w*BSRM.LA_deri_factor*Ns*Ns*BSRM.IA_X)/(BSRM.LA_facotr*Ns*Ns);
    iyA_k4=(CTRL.UA_Y-BSRM.Rs*BSRM.IA_Y-BSRM.mech_w*BSRM.LA_deri_factor*Ns*Ns*BSRM.IA_Y)/(BSRM.LA_facotr*Ns*Ns);

    imB_k4=(CTRL.UB-BSRM.Rm*BSRM.IB-BSRM.mech_w*BSRM.LB_deri_factor*2*Nm*Nm*BSRM.IB)/(BSRM.LB_facotr*2*Nm*Nm);
    ixB_k4=(CTRL.UB_X-BSRM.Rs*BSRM.IB_X-BSRM.mech_w*BSRM.LB_deri_factor*Ns*Ns*BSRM.IB_X)/(BSRM.LB_facotr*Ns*Ns);
    iyB_k4=(CTRL.UB_Y-BSRM.Rs*BSRM.IB_Y-BSRM.mech_w*BSRM.LB_deri_factor*Ns*Ns*BSRM.IB_Y)/(BSRM.LB_facotr*Ns*Ns);   

    imC_k4=(CTRL.UC-BSRM.Rm*BSRM.IC-BSRM.mech_w*BSRM.LC_deri_factor*2*Nm*Nm*BSRM.IC)/(BSRM.LC_facotr*2*Nm*Nm);
    ixC_k4=(CTRL.UC_X-BSRM.Rs*BSRM.IC_X-BSRM.mech_w*BSRM.LC_deri_factor*Ns*Ns*BSRM.IC_X)/(BSRM.LC_facotr*Ns*Ns);
    iyC_k4=(CTRL.UC_Y-BSRM.Rs*BSRM.IC_Y-BSRM.mech_w*BSRM.LC_deri_factor*Ns*Ns*BSRM.IC_Y)/(BSRM.LC_facotr*Ns*Ns);
    
    w_k=(w_k1+2*w_k2+2*w_k2+w_k4)/6;
    x_k=(x_k1+2*x_k2+2*x_k3+x_k4)/6;
    y_k=(y_k1+2*y_k2+2*y_k3+y_k4)/6;
    x_v_k=(x_v_k1+2*x_v_k2+2*x_v_k3+x_v_k4)/6;
    y_v_k=(y_v_k1+2*y_v_k2+2*y_v_k3+y_v_k4)/6;;
    theta_k=(theta_k1+2*theta_k2+2*theta_k3+theta_k4)/6;
    imA_k=(imA_k1+2*imA_k2+2*imA_k3+imA_k4)/6;
    ixA_k=(ixA_k1+2*ixA_k2+2*ixA_k3+ixA_k4)/6;
    iyA_k=(iyA_k1+2*iyA_k2+2*iyA_k3+iyA_k4)/6;
    imB_k=(imB_k1+2*imB_k2+2*imB_k3+imB_k4)/6;
    ixB_k=(ixB_k1+2*ixB_k2+2*ixB_k3+ixB_k4)/6;
    iyB_k=(iyB_k1+2*iyB_k2+2*iyB_k3+iyB_k4)/6; 
    imC_k=(imC_k1+2*imC_k2+2*imC_k3+imC_k4)/6;
    ixC_k=(ixC_k1+2*ixC_k2+2*ixC_k3+ixC_k4)/6;
    iyC_k=(iyC_k1+2*iyC_k2+2*iyC_k3+iyC_k4)/6;


    // FINAL STEP
    BSRM.mech_angle=f_angle+ TIME_EACH_STEP*theta_k;
    BSRM.mech_w=f_w+ TIME_EACH_STEP*w_k;
    BSRM.x_v=fx_v+ TIME_EACH_STEP*x_v_k;
    BSRM.y_v=fy_v+ TIME_EACH_STEP*y_v_k;
    BSRM.x_displacement=fx_dis+ TIME_EACH_STEP*x_k;//    BSRM.x_v=BSRM.x_v+ TIME_EACH_STEP*x_k1;
    BSRM.y_displacement=fy_dis+ TIME_EACH_STEP*y_k;//    BSRM.y_v=BSRM.y_v+ TIME_EACH_STEP*y_k1;

    BSRM.IA=fma+ TIME_EACH_STEP*imA_k;
    BSRM.IA_X=f1a+ TIME_EACH_STEP*ixA_k;
    BSRM.IA_Y=f2a+ TIME_EACH_STEP*iyA_k;
    BSRM.IB=fmb+ TIME_EACH_STEP*imB_k;
    BSRM.IB_X=f1b+ TIME_EACH_STEP*ixB_k;
    BSRM.IB_Y=f2b+ TIME_EACH_STEP*iyB_k;
    BSRM.IC=fmc+ TIME_EACH_STEP*imC_k;
    BSRM.IC_X=f1c+ TIME_EACH_STEP*ixC_k;
    BSRM.IC_Y=f2c+ TIME_EACH_STEP*iyC_k;

    dynamics();
}

int machine_simulation(int step){ 
    
    /* Load Torque */
    // BSRM.Tload = 0 * sign(BSRM.rpm); // No-load test
    // BSRM.Tload = BSRM.Tem; // Blocked-rotor test
    
    if (step<(NUMBER_OF_STEPS/2)){
        BSRM.Tload = 0.01 * friction_factor * (BSRM.rpm);
    }
    else{
        BSRM.Tload = 0.060 * sign(BSRM.rpm)+  0.01 * friction_factor * (BSRM.rpm);
    }
    
   
   
    BSRM.X_load = 2 ; //1*sin(2*pi*1*BSRM.TIME);
    BSRM.Y_load = 0 ; 
    // solve for BSRM.x with force and torque, inputs STEP
    Range_Kuta(step);
    //ALL OF THE IMFORMATION IS STORED IN BSRM.

    // detect bad simulation
    if(isNumber(BSRM.rpm)){
        return false;
    }
    else{
        printf("BSRM.rpm is %g\n", BSRM.rpm);
        return true;        
    }
}

void Machine_init(){

    BSRM.m = 0.38;
    BSRM.Rm = 0.4;
    BSRM.Rs = 0.2;
    BSRM.Um = 12;
    BSRM.Us = 6;
    BSRM.X_load = 0;
    BSRM.Y_load = 0;
    BSRM.Tload = 0;
    BSRM.mech_w = 0;
    BSRM.mech_angle = pi/24;
    BSRM.TIME = 0;
    BSRM.x_displacement = 0;
    BSRM.y_displacement = 0;
    BSRM.x_v = 0;
    BSRM.y_v = 0;
    BSRM.IA = 0;
    BSRM.IA_X = 0;
    BSRM.IA_Y = 0;
    BSRM.IA_X_pre=0;
    BSRM.IA_Y_pre=0;
    BSRM.IB = 0;
    BSRM.IB_X = 0;
    BSRM.IB_Y = 0;
    BSRM.IB_X_pre=0;
    BSRM.IB_Y_pre=0;
    BSRM.IC = 0;
    BSRM.IC_X = 0;
    BSRM.IC_Y = 0;
    BSRM.IC_X_pre=0;
    BSRM.IC_Y_pre=0;

    angle_trans(BSRM.mech_angle);
    dynamics(); // 0s

}

void measurement_slow(){
    static int measure_judge_slow=0;
    measure_judge_slow++;
    if(measure_judge_slow == 200 ){//NUMBER_OF_STEPS/MEASURE_FRE
        measure_judge_slow=0;
        
        CTRL.measure_count++;
        CTRL.mech_angle = BSRM.mech_angle;
        CTRL.x_displacement = BSRM.x_displacement;
        CTRL.y_displacement = BSRM.y_displacement;
        CTRL.rpm=BSRM.rpm;

        CTRL.Kf1_A=BSRM.Kf1_A;
        CTRL.Kf2_A=BSRM.Kf2_A;
        CTRL.Kf1_B=BSRM.Kf1_B;
        CTRL.Kf2_B=BSRM.Kf2_B;
        CTRL.Kf1_C=BSRM.Kf1_C;
        CTRL.Kf2_C=BSRM.Kf2_C;

        CTRL.LA_facotr=BSRM.LA_facotr;
        CTRL.LB_facotr=BSRM.LB_facotr;
        CTRL.LC_facotr=BSRM.LC_facotr;  
        CTRL.LA_deri_factor=BSRM.LA_deri_factor;
        CTRL.LB_deri_factor=BSRM.LB_deri_factor;
        CTRL.LC_deri_factor=BSRM.LC_deri_factor;

        CTRL.angle_A=BSRM.angle_A;
        CTRL.angle_B=BSRM.angle_B;
        CTRL.angle_C=BSRM.angle_C;

    }

}

void measurement_quick(){
   
    BSRM.rpm=BSRM.mech_w*60/2/pi;
}


int main(){
    
    printf("NUMBER_OF_STEPS: %d\n\n", NUMBER_OF_STEPS);

    /* Initialization */
    Machine_init();
    CTRL_init();
    //   obesever_init();

    FILE *fw; 
    fw = fopen("info.txt", "w");//creat a file named info.txt. If exist, clear all.
    printf("%s\n", "data.txt");   
    write_header_to_file(fw);

    /* MAIN LOOP */
    //record the running time
    clock_t begin_time, end_time;
    begin_time = clock();

    int step; // _ for the outer iteration
    
    // Frequence @ 2000000, control frequence @ 10000
    for(step=0;step<(NUMBER_OF_STEPS/2);++step){
        
        // printf("%d\n", _);
        BSRM.TIME=step*TIME_EACH_STEP;
        /* Command (Speed or Position) */
        // BSRM.rpm_cmd = 2000;
        // if(CTRL.timebase>10){
        //     BSRM.rpm_cmd = 2000;S
        // }


        /* Simulated BSRM */
        if(machine_simulation(step)){ 
            printf("Break the loop.\n");
            break;
        }

        // write data to file  
        //frequence  WRITE_FRE 50000
        write_data_to_file(fw);
        
        //frequence  MEASURE_FRE  for digital signal processer
        measurement_slow();
        
        //high fre feedback for analog signal process
        measurement_quick();
        
        /* in DSP */
        //frequence  CTRL_FRE 10000
        control(step);
        

        //this part is realized by anolog circuit @ a very high frequency
        //commutation logical judgement and circuit control

        if (step%4==0){
            circuit_main_A();
            circuit_main_B();
            circuit_main_C();
        }
         
    }
    
    
    end_time = clock();
    printf( "Simulation in C costs %g sec.\n", (double)(end_time - begin_time)/CLOCKS_PER_SEC);

    fclose(fw);
    /* Fade out */
    //while(getch() != '\n') ;
    //system("python ./BSRMPlot.py"); 
    //getch();
    //system("pause");
    //system("exit");
    //return 0; 
}

//  please use the command goes  
//  gcc main.c controller.c observer.c -o info
//  then run use
//  .\info.exe 
//  python BSRMPlot.py


//  del info.exe
//  del info.txt 
//  del.info.dat
