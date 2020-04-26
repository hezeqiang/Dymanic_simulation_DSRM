#include "BSRMSim.h"

struct BSRMSimulated BSRM;

void write_header_to_file(FILE *fw){// include the explaination of data imformation
    // no space is allowed!
    fprintf(fw, "x0(id)[A],x1(iq)[A],x2(speed)[rad/s],x3(position)[rad],ud_cmd[V],uq_cmd[V],id_cmd[A],id_err[A],iq_cmd[A],iq_err[A],|eemf|[V],eemf_be[V],theta_d[rad],theta_d__eemf[rad],mismatch[rad],sin(mismatch)[rad],OB_POS,sin(ER_POS),OB_EEMF_BE,error(OB_EEMF),ob.xEEMF_dummy[0],error(xEEMF_dummy),OB_OMG,er_omg\n");    
    {
        FILE *fw2;
        fw2 = fopen("info.dat", "w");   //create a new file named info.dat
        fprintf(fw2, "TS,DOWN_SAMPLE,DATA_FILE_NAME\n");
        fprintf(fw2, "%g, %d, %s\n", TS, WRITE_SAEMPL, "data.txt");
        //%f   output as format float 3.1415926
        //%e   output like 3.141593e+000
        //%g   automatically choose output format 3.14159
        //%d   output like 3
        //%s   string
        fclose(fw2);
    }
}

void write_data_to_file(FILE *fw){
    static int bool_animate_on = false;
    static int j=0,jj=0; // j,jj for write sampling

    // if(CTRL.timebase>20)
    {
        if(++j == WRITE_SAMPLE)
        {
            j=0;
            fprintf(fw, "%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g\n",
                    ACM.x[0], ACM.x[1], ACM.x[2], ACM.x[3], CTRL.ud_cmd, CTRL.uq_cmd, 
                    CTRL.id_cmd, CTRL.id__fb-CTRL.id_cmd, CTRL.iq_cmd, CTRL.iq__fb-CTRL.iq_cmd, ACM.eemf_q, ACM.eemf_be,
                    ACM.theta_d, ACM.theta_d__eemf,ACM.theta_d-ACM.theta_d__eemf,sin(ACM.theta_d-ACM.theta_d__eemf),
                    OB_POS, sin(ACM.theta_d-OB_POS), OB_EEMF_BE, ACM.eemf_be-OB_EEMF_BE, ob.xEEMF_dummy[1], OB_EEMF_BE-ob.xEEMF_dummy[1], OB_OMG, ACM.omg_elec-OB_OMG
                    );
        }
    }

    // if(bool_animate_on==false){
    //     bool_animate_on = true;
    //     printf("Start ACMAnimate\n");
    //     system("start python ./ACMAnimate.py"); 
    // }
}

void angle_trans(double mech_angle){
    if (fmod(mech_angle,pi/4)<pi/8){BSRM.angle_A=fmod(mech_angle,pi/4);}
    else {BSRM.angle_A=fmod(mech_angle,pi/4)-pi/4;}//A PHASE
    if (fmod(mech_angle+pi/12,pi/4)<pi/8){BSRM.angle_B=fmod(mech_angle+pi/12,pi/4);}
    else {BSRM.angle_B=fmod(mech_angle+pi/12,pi/4)-pi/4;}
    if (fmod(mech_angle-pi/12,pi/4)<pi/8){BSRM.angle_C=fmod(mech_angle-pi/1,pi/4);}
    else {BSRM.angle_C=fmod(mech_angle-pi/12,pi/4)-pi/4;}
}

void cal_force_A(double angle_A,double IA,double IA_X,double IA_Y){
    if (IA!=0.0){
        cal_Kf_A(angle_A);
        BSRM.x_force_A=IA*(BSRM.Kf1_A*IA_X+BSRM.Kf2_A*IA_Y);
        BSRM.y_force_A=IA*(BSRM.Kf2_A*IA_X+BSRM.Kf1_A*IA_Y);
    }
    else{
        BSRM.x_force_A=0;
        BSRM.y_force_A=0;
    }
}
void cal_force_B(double angle_B,double IB,double IB_X,double IB_Y){
    if (IB!=0.0){
        cal_Kf_A(angle_B);
        BSRM.x_force_B=IB*(BSRM.Kf1_B*IB_X+BSRM.Kf2_B*IB_Y);
        BSRM.y_force_B=IB*(BSRM.Kf2_B*IB_X+BSRM.Kf1_B*IB_Y);
    }
    else{
        BSRM.x_force_B=0;
        BSRM.y_force_B=0;
    }
}
void cal_force_C(double angle_C,double IC,double IC_X,double IC_Y){
    if (IC!=0.0){
        cal_Kf_A(angle_C);
        BSRM.x_force_C=IC*(BSRM.Kf1_C*IC_X+BSRM.Kf2_C*IC_Y);
        BSRM.y_force_C=IC*(BSRM.Kf2_C*IC_X+BSRM.Kf1_C*IC_Y);
    }
    else{
        BSRM.x_force_C=0;
        BSRM.y_force_C=0;
    }
}
    
void cal_Kf_A(double angle){
    BSRM.Kf1_A=Nm*Ns*(u0*H_STACK*RADIUS*(pi-12*angle)/6/(l_0*l_0)+4*u0*H_STACK*RADIUS*(2*RADIUS*angle+l_0)*angle/(pi*RADIUS*RADIUS*angle*angle-(pi+2)*RADIUS*angle*l_0+2*l_0*l_0));
    BSRM.Kf2_A=Nm*Ns*(u0*H_STACK*RADIUS*(pi-12*angle)*angle/12/(l_0*l_0)-2*u0*H_STACK/l_0+4*u0*H_STACK*RADIUS*(2*RADIUS*angle+l_0)*angle/(pi*RADIUS*RADIUS*angle*angle-(pi+2)*RADIUS*angle*l_0+2*l_0*l_0));
    //h_stack,r,l_air,Nm,Ns
}
void cal_Kf_B(double angle){
    BSRM.Kf1_B=Nm*Ns*(u0*H_STACK*RADIUS*(pi-12*angle)/6/(l_0*l_0)+4*u0*H_STACK*RADIUS*(2*RADIUS*angle+l_0)*angle/(pi*RADIUS*RADIUS*angle*angle-(pi+2)*RADIUS*angle*l_0+2*l_0*l_0));
    BSRM.Kf2_B=Nm*Ns*(u0*H_STACK*RADIUS*(pi-12*angle)*angle/12/(l_0*l_0)-2*u0*H_STACK/l_0+4*u0*H_STACK*RADIUS*(2*RADIUS*angle+l_0)*angle/(pi*RADIUS*RADIUS*angle*angle-(pi+2)*RADIUS*angle*l_0+2*l_0*l_0));
    //h_stack,r,l_air,Nm,Ns
}
void cal_Kf_C(double angle){
    BSRM.Kf1_C=Nm*Ns*(u0*H_STACK*RADIUS*(pi-12*angle)/6/(l_0*l_0)+4*u0*H_STACK*RADIUS*(2*RADIUS*angle+l_0)*angle/(pi*RADIUS*RADIUS*angle*angle-(pi+2)*RADIUS*angle*l_0+2*l_0*l_0));
    BSRM.Kf2_C=Nm*Ns*(u0*H_STACK*RADIUS*(pi-12*angle)*angle/12/(l_0*l_0)-2*u0*H_STACK/l_0+4*u0*H_STACK*RADIUS*(2*RADIUS*angle+l_0)*angle/(pi*RADIUS*RADIUS*angle*angle-(pi+2)*RADIUS*angle*l_0+2*l_0*l_0));
    //h_stack,r,l_air,Nm,Ns
}   

 
void cal_torque_A(double IA,double IA_X,double IA_Y,double angle_A){
    if (IA!=0){
        if(angle_A<0){
            BSRM.Tem_A=(Nm*Nm*IA*IA+Ns*Ns*IA_Y*IA_Y/2+Ns*Ns*IA_Y*IA_Y/2)*(2*u0*RADIUS*H_STACK/l_0-4*u0*H_STACK*RADIUS*(2*RADIUS*angle_A-l_0)/(pi*RADIUS*RADIUS*angle_A*angle_A-(pi+2)*RADIUS*angle_A*l_0+2*l_0*l_0));
            }
        else {
            BSRM.Tem_A=(Nm*Nm*IA*IA+Ns*Ns*IA_Y*IA_Y/2+Ns*Ns*IA_Y*IA_Y/2)*(-2*u0*RADIUS*H_STACK/l_0+4*u0*H_STACK*RADIUS*(2*RADIUS*angle_A+l_0)/(pi*RADIUS*RADIUS*angle_A*angle_A+(pi+2)*RADIUS*angle_A*l_0+2*l_0*l_0));
            } 
    }  
    else  
        BSRM.Tem_A=0;       
}

void cal_torque_B(double IB,double IB_X,double IB_Y,double angle_B){
    if (IB!=0){
       if(angle_B<0){
            BSRM.Tem_B=(Nm*Nm*IB*IB+Ns*Ns*IB_Y*IB_Y/2+Ns*Ns*IB_Y*IB_Y/2)*(2*u0*RADIUS*H_STACK/l_0-4*u0*H_STACK*RADIUS*(2*RADIUS*angle_B-l_0)/(pi*RADIUS*RADIUS*angle_B*angle_B-(pi+2)*RADIUS*angle_B*l_0+2*l_0*l_0));
            }
        else {
            BSRM.Tem_B=(Nm*Nm*IB*IB+Ns*Ns*IB_Y*IB_Y/2+Ns*Ns*IB_Y*IB_Y/2)*(-2*u0*RADIUS*H_STACK/l_0+4*u0*H_STACK*RADIUS*(2*RADIUS*angle_B+l_0)/(pi*RADIUS*RADIUS*angle_B*angle_B+(pi+2)*RADIUS*angle_B*l_0+2*l_0*l_0));
            }     }
    else {
        BSRM.Tem_B=0;
    }
}

void cal_torque_C(double IC,double IC_X,double IC_Y,double angle_C){
    if (IC!=0){
        if(angle_C<0){
            BSRM.Tem_C=(Nm*Nm*IC*IC+Ns*Ns*IC_Y*IC_Y/2+Ns*Ns*IC_Y*IC_Y/2)*(2*u0*RADIUS*H_STACK/l_0-4*u0*H_STACK*RADIUS*(2*RADIUS*angle_C-l_0)/(pi*RADIUS*RADIUS*angle_C*angle_C-(pi+2)*RADIUS*angle_C*l_0+2*l_0*l_0));
            }
        else {
            BSRM.Tem_C=(Nm*Nm*IC*IC+Ns*Ns*IC_Y*IC_Y/2+Ns*Ns*IC_Y*IC_Y/2)*(-2*u0*RADIUS*H_STACK/l_0+4*u0*H_STACK*RADIUS*(2*RADIUS*angle_C+l_0)/(pi*RADIUS*RADIUS*angle_C*angle_C+(pi+2)*RADIUS*angle_C*l_0+2*l_0*l_0));
            } 
    }
    else {
        BSRM.Tem_C=0;
    }
}

void cal_L_factor_A(double angle){
    BSRM.LA_facotr=u0*H_STACK*RADIUS*(pi-12*angle)/6/l_0+4*u0*H_STACK/pi*log(pi*RADIUS*RADIUS*angle*angle+(pi+2)*RADIUS*angle*l_0+2*l_0*l_0/(2*l_0*l_0))-8*u0*H_STACK/pi/(pi-2)*log((pi*RADIUS*RADIUS*angle*angle-(pi+2)*RADIUS*angle*l_0+2*l_0*l_0)/2/(RADIUS*angle+l_0)/(RADIUS*angle+l_0));  
}

void cal_L_factor_B(double angle){   
    BSRM.LB_facotr=u0*H_STACK*RADIUS*(pi-12*angle)/6/l_0+4*u0*H_STACK/pi*log(pi*RADIUS*RADIUS*angle*angle+(pi+2)*RADIUS*angle*l_0+2*l_0*l_0/(2*l_0*l_0))-8*u0*H_STACK/pi/(pi-2)*log((pi*RADIUS*RADIUS*angle*angle-(pi+2)*RADIUS*angle*l_0+2*l_0*l_0)/2/(RADIUS*angle+l_0)/(RADIUS*angle+l_0));  
}

void cal_L_factor_C(double angle){   
    BSRM.LC_facotr=u0*H_STACK*RADIUS*(pi-12*angle)/6/l_0+4*u0*H_STACK/pi*log(pi*RADIUS*RADIUS*angle*angle+(pi+2)*RADIUS*angle*l_0+2*l_0*l_0/(2*l_0*l_0))-8*u0*H_STACK/pi/(pi-2)*log((pi*RADIUS*RADIUS*angle*angle-(pi+2)*RADIUS*angle*l_0+2*l_0*l_0)/2/(RADIUS*angle+l_0)/(RADIUS*angle+l_0));  
}



void cal_L_deri_factor_A(double angle,double angle_abs){   
    BSRM.LA_deri_factor=2*u0*H_STACK*RADIUS/l_0+(4*u0*H_STACK/pi-8*u0*H_STACK/pi/(pi-2))*(2*pi*RADIUS*RADIUS*angle-RADIUS*l_0*(pi+2))/(pi*RADIUS*RADIUS*angle*angle+(pi+2)*RADIUS*angle*l_0+2*l_0*l_0)-32*u0*H_STACK/pi/(pi-2)*RADIUS/2/(l_0-RADIUS*angle);
    if(angle>=0){BSRM.LA_deri_factor=-BSRM.LA_deri_factor;}
}

void cal_L_deri_factor_B(double angle,double angle_abs){
    BSRM.LB_deri_factor=2*u0*H_STACK*RADIUS/l_0+(4*u0*H_STACK/pi-8*u0*H_STACK/pi/(pi-2))*(2*pi*RADIUS*RADIUS*angle-RADIUS*l_0*(pi+2))/(pi*RADIUS*RADIUS*angle*angle+(pi+2)*RADIUS*angle*l_0+2*l_0*l_0)-32*u0*H_STACK/pi/(pi-2)*RADIUS/2/(l_0-RADIUS*angle);
    if(angle>=0){BSRM.LB_deri_factor=-BSRM.LB_deri_factor;}
}

void cal_L_deri_factor_C(double angle,double angle_abs){
    BSRM.LC_deri_factor=2*u0*H_STACK*RADIUS/l_0+(4*u0*H_STACK/pi-8*u0*H_STACK/pi/(pi-2))*(2*pi*RADIUS*RADIUS*angle-RADIUS*l_0*(pi+2))/(pi*RADIUS*RADIUS*angle*angle+(pi+2)*RADIUS*angle*l_0+2*l_0*l_0)-32*u0*H_STACK/pi/(pi-2)*RADIUS/2/(l_0-RADIUS*angle);
    if(angle>=0){BSRM.LC_deri_factor=-BSRM.LC_deri_factor;}
}



/* Simple Model based on the differential equaiton */
void RK_dynamics(Tem, x_f, y_f){
    BSRM.Tload;
    BSRM.X_load;
    BSRM.Y_load;

}

void Range_Kuta(int TIME, double Tem, double x_f ,double y_f){

    int i;
    TIME_EACH_STEP;
    
    RK_dynamics(Tem, x_f, y_f); // timer.t,
    for(i=0;i<NS;++i){        
        k1[i] = fx[i] * hs;
        xk[i] = x[i] + k1[i]*0.5;
    }
    
    RK_dynamics(t, xk, fx); // timer.t+hs/2., 
    for(i=0;i<NS;++i){        
        k2[i] = fx[i] * hs;
        xk[i] = x[i] + k2[i]*0.5;
    }
    
    RK_dynamics(t, xk, fx); // timer.t+hs/2., 
    for(i=0;i<NS;++i){        
        k3[i] = fx[i] * hs;
        xk[i] = x[i] + k3[i];
    }
    
    RK_dynamics(t, xk, fx); // timer.t+hs, 
    for(i=0;i<NS;++i){        
        k4[i] = fx[i] * hs;
        x[i] = x[i] + (k1[i] + 2*(k2[i] + k3[i]) + k4[i])/6.0;

        // derivatives
        ACM.x_dot[i] = (k1[i] + 2*(k2[i] + k3[i]) + k4[i])/6.0 / hs; 
    }
}

int machine_simulation(step){
   
    //the state of BSRM is given here
    
    /* Load Torque */
    // BSRM.Tload = 0 * sign(BSRM.rpm); // No-load test
    // BSRM.Tload = BSRM.Tem; // Blocked-rotor test
    BSRM.Tload = 2 * sign(BSRM.rpm)+friction_factor*(BSRM.rpm);

    /* raduis force in both axis and Tem */
 
    angle_trans(BSRM.mech_angle);

    cal_force_A(fbs(BSRM.angle_A),BSRM.IA,BSRM.IA_X,BSRM.IA_Y);
    cal_force_B(fbs(BSRM.angle_B),BSRM.IB,BSRM.IB_X,BSRM.IB_Y);
    cal_force_C(fbs(BSRM.angle_C),BSRM.IC,BSRM.IC_X,BSRM.IC_Y);

    cal_torque_A(BSRM.angle_A,BSRM.IA,BSRM.IA_X,BSRM.IA_Y);
    cal_torque_B(BSRM.angle_B,BSRM.IB,BSRM.IB_X,BSRM.IB_Y);
    cal_torque_C(BSRM.angle_C,BSRM.IC,BSRM.IC_X,BSRM.IC_Y);

    cal_L_factor_A(fbs(BSRM.angle_A));
    cal_L_factor_B(fbs(BSRM.angle_B));
    cal_L_factor_C(fbs(BSRM.angle_C));

    cal_L_deri_factor_A(-fbs(BSRM.angle_A),fbs(BSRM.angle_A));
    cal_L_deri_factor_B(-fbs(BSRM.angle_B),fbs(BSRM.angle_B));
    cal_L_deri_factor_C(-fbs(BSRM.angle_C),fbs(BSRM.angle_C));

    BSRM.Tem=BSRM.Tem_A+BSRM.Tem_B+BSRM.Tem_C;
    BSRM.x_force= BSRM.x_force_A+(BSRM.x_force_B+BSRM.x_force_C)*sqrt3/2+(-BSRM.y_force_B+BSRM.y_force_C)*1/2;
    BSRM.y_force= BSRM.y_force_A+(BSRM.y_force_B+BSRM.y_force_C)*sqrt3/2+(BSRM.x_force_B-BSRM.x_force_C)*1/2;
    
    // solve for BSRM.x with force and torque as inputs
    Range_Kuta(step,BSRM.Tem,BSRM.x_force,BSRM.y_force);

    // rotor position
    ACM.theta_d = ACM.x[3];
    if(ACM.theta_d > M_PI){
        ACM.theta_d -= 2*M_PI;
    }else if(ACM.theta_d < -M_PI){
        ACM.theta_d += 2*M_PI; // 反转！
    }
    ACM.x[3] = ACM.theta_d;

    // currents
    ACM.id  = ACM.x[0];
    ACM.iq  = ACM.x[1];
    ACM.ial = MT2A(ACM.id, ACM.iq, cos(ACM.theta_d), sin(ACM.theta_d));
    ACM.ibe = MT2B(ACM.id, ACM.iq, cos(ACM.theta_d), sin(ACM.theta_d));

    // speed
    ACM.omg_elec = ACM.x[2];
    ACM.rpm = ACM.x[2] * 60 / (2 * M_PI * ACM.npp);

    // detect bad simulation
    if(isNumber(ACM.rpm)){
        return false;
    }else{
        printf("ACM.rpm is %g\n", ACM.rpm);
        return true;        
    }
}


int main(){
    
    printf("NUMBER_OF_STEPS: %d\n\n", NUMBER_OF_STEPS);

    /* Initialization */
    Machine_init();
    CTRL_init();
 //   sm_init();
 //   obesever_init();

    FILE *fw;
    fw = fopen("data.txt", "w");
    printf("%s\n", "data.txt");
    printf("%s\n", "data.txt");
    printf("%s\n", "data.txt");    
    write_header_to_file(fw);

    /* MAIN LOOP */
    //record the running time
    clock_t begin, end;
    begin = clock();

    int step; // _ for the outer iteration
    float timebase;
    int dfe_counter=0; // dfe_counter for write frequency execution
    

    for(step=0;step<NUMBER_OF_STEPS;++step){
        
        BSRM.X_load=10;
        BSRM.Y_load=10;
        // printf("%d\n", _);
        timebase=step*TIME_EACH_STEP;
        /* Command (Speed or Position) */
        // cmd_fast_speed_reversal(CTRL.timebase, 5, 5, 1500); // timebase, instant, interval, rpm_cmd
        cmd_fast_speed_reversal(CTRL.timebase, 5, 5, 100); // timebase, instant, interval, rpm_cmd
        // BSRM.rpm_cmd = 500;
        // if(CTRL.timebase>10){
        //     BSRM.rpm_cmd = 2000;
        // }

        /* Load Torque */
        // BSRM.Tload = 0 * sign(BSRM.rpm); // No-load test
        // BSRM.Tload = BSRM.Tem; // Blocked-rotor test
        BSRM.Tload = 2 * sign(BSRM.rpm)+friction_factor*(BSRM.rpm);

        /* Simulated BSRM */
        if(machine_simulation(step)){ 
            printf("Break the loop.\n");
            break;
        }

        if(++dfe_counter == TS_UPSAMPLING_FREQ_EXE_INVERSE){
            dfe_counter = 0;

            /* Time in DSP */
            CTRL.timebase += TS;

            measurement();

            observation();

            write_data_to_file(fw);

            control(ACM.rpm_cmd, 0);
        }

        inverter_model();
    }
    end = clock(); printf("The simulation in C costs %g sec.\n", (double)(end - begin)/CLOCKS_PER_SEC);
    fclose(fw);

    /* Fade out */
    system("python ./BSRMPlot.py"); 
    // getch();
    // system("pause");
    // system("exit");
    return 0; 
}