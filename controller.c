#include "BSRMSim.h"
#include "controller.h"


struct BSRMSimulated BSRM;
struct ControllerForExperiment CTRL;

/* Incremental PID Control */
//Tem
double PID_Speed(struct PID_Reg *r, double error){
    // incremental part, available to recalculate for spcifical cut-off frequence
    // the instance is a typical PID control with a limit.
    // in order to satisfy controlling request, the error= speed-speed.command
    // refer to https://blog.csdn.net/kilotwo/article/details/79829669
    if ( fbs(error)<r->cutoff )
        r->output_incre=r->Kp*(error-r->pre_error)+r->Ki*error+r->Kd*(error-2*r->pre_error+r->prepre_error);
    else{
        r->output_incre=r->Kp*(error-r->pre_error)+0*r->Ki*error+r->Kd*(error-2*r->pre_error+r->prepre_error);
    }  
    //the absolute output
    r->output=r->output_incre + r->pre_output;
    r->pre_output=r->output;//pass the previous value to r->pre_output first then consider the limitation

    // r->limit=pi/24, the maximum advanced open angle
    if(r->output > r->limit)  
        r->output = r->limit;
    else if(r->output <= -r->limit)
        r->output = -r->limit;
    
    r->prepre_error=r->pre_error;
    r->pre_error=error;
    return r->output;

}

//Radial force
double PID_Dis(struct PID_Reg *r, double error){
    // incremental part, available to recalculate for spcifical cut-off frequence
    // the instance is a typical PID control with a limit.
    // in order to satisfy controlling request, the error= speed-speed.command
    // refer to https://blog.csdn.net/kilotwo/article/details/79829669
    if ( fbs(error)<r->cutoff )
        r->output_incre=r->Kp*(error-r->pre_error)+r->Ki*error+r->Kd*(error-2*r->pre_error+r->prepre_error); //CTRL_FRE
    else{
        r->output_incre=r->Kp*(error-r->pre_error)+0*r->Ki*error+r->Kd*(error-2*r->pre_error+r->prepre_error);
    }  
    //the absolute output
    r->output=r->output_incre + r->pre_output;
    r->prepre_output=r->pre_output;
    r->pre_output=r->output;//pass the previous value to r->pre_output first then consider the limitation

    // r->limit=pi/24, the maximum advanced open angle
    if(r->output > r->limit)  
        r->output = r->limit;
    else if(r->output <= -r->limit)
        r->output = -r->limit;
    
    r->prepre_error=r->pre_error;
    r->pre_error=error;
    return r->output;

}


/* Initialization */
void CTRL_init(){
    //initial measurement before motor start 
    CTRL.mech_angle = BSRM.mech_angle;
    CTRL.rpm=BSRM.rpm;
    CTRL.m=BSRM.m;
    CTRL.rpm_cmd = 2000 ;// rpm command
    CTRL.advance_angle=0;
    CTRL.main_current= 15;
    CTRL.current_hystersis= 0.1 ;
    CTRL.ctrl_count=0;
    CTRL.MAX_DIS=8;
    
    CTRL.Rm=BSRM.Rm;
    CTRL.Rs=BSRM.Rs;
    CTRL.Um=BSRM.Um;
    CTRL.Us=BSRM.Us;


    CTRL.x_displacement = BSRM.x_displacement;
    CTRL.y_displacement = BSRM.y_displacement;

    CTRL.IA_X=BSRM.IA_X;
    CTRL.IA_Y=BSRM.IA_Y;
    CTRL.IB_X=BSRM.IB_Y;
    CTRL.IB_X=BSRM.IB_Y;
    CTRL.IC_X=BSRM.IC_X;
    CTRL.IC_Y=BSRM.IC_Y;


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


    CTRL.x_force_A=0;
    CTRL.y_force_A=0;
    CTRL.x_force_B=0;
    CTRL.y_force_B=0;
    CTRL.x_force_C=0;
    CTRL.y_force_C=0;
    CTRL.x_force=0;
    CTRL.y_force=0;

    CTRL.UA=0;
    CTRL.UB=0;
    CTRL.UC=0;
    CTRL.UA_X=0;
    CTRL.UB_X=0;
    CTRL.UC_X=0;
    CTRL.UA_Y=0;
    CTRL.UB_Y=0;
    CTRL.UC_Y=0;

    CTRL.angle_A=BSRM.angle_A;
    CTRL.angle_B=BSRM.angle_B;
    CTRL.angle_C=BSRM.angle_C;

    CTRL.measure_count=0;

    // ver. IEMDC
    CTRL.PID_speed.Kp = SPEED_LOOP_PID_PROPORTIONAL_GAIN;
    CTRL.PID_speed.Ki = SPEED_LOOP_PID_INTEGRAL;
    CTRL.PID_speed.Kd = SPEED_LOOP_PID_DIREVATIVE;
    CTRL.PID_speed.limit = SPEED_LOOP_LIMIT_ANGLE;
    CTRL.PID_speed.prepre_error= 0.0;
    CTRL.PID_speed.pre_error= 0.0;

    CTRL.PID_speed.pre_output=0.0;
    CTRL.PID_speed.output=0.0;
    CTRL.PID_speed.cutoff=300;
    printf("Speed PID: Kp=%f, Ki=%f, Kd=%f, limit=%f Nm\n", CTRL.PID_speed.Kp, CTRL.PID_speed.Ki,CTRL.PID_speed.Kd, CTRL.PID_speed.limit);

    CTRL.PID_disx.Kp = DIS_LOOP_PID_PROPORTIONAL_GAIN;
    CTRL.PID_disx.Ki = DIS_LOOP_PID_INTEGRAL;
    CTRL.PID_disx.Kd = DIS_LOOP_PID_DIREVATIVE;
    CTRL.PID_disx.Kd_up = 0.01;
    CTRL.PID_disx.Kd_down = 0.000001;
    CTRL.PID_disx.limit = DIS_LOOP_LIMIT;
    
    CTRL.PID_disx.prepre_error= 0.0;
    CTRL.PID_disx.pre_error= 0.0;
    CTRL.PID_disx.pre_output=0.0;
    CTRL.PID_disx.prepre_output=0.0;
    CTRL.PID_disx.output=0.0;
    CTRL.PID_disx.cutoff=0.00001;
    printf("displacement x PID: Kp=%f, Ki=%f, Kd=%f, limit=%f Nm\n",CTRL.PID_disx.Kp, CTRL.PID_disx.Ki,CTRL.PID_disx.Kd, CTRL.PID_disx.limit);

    CTRL.PID_disy.Kp = DIS_LOOP_PID_PROPORTIONAL_GAIN;
    CTRL.PID_disy.Ki = DIS_LOOP_PID_INTEGRAL;
    CTRL.PID_disy.Kd = DIS_LOOP_PID_DIREVATIVE;
    CTRL.PID_disy.Kd_up = 0.01;
    CTRL.PID_disy.Kd_down = 0.000001;
    CTRL.PID_disy.limit = DIS_LOOP_LIMIT;
    
    CTRL.PID_disy.prepre_error= 0.0;
    CTRL.PID_disy.pre_error= 0.0;
    CTRL.PID_disy.pre_output=0.0;
    CTRL.PID_disy.output=0.0;
    CTRL.PID_disy.prepre_output=0.0;
    CTRL.PID_disy.cutoff=0.00001;
    printf("displacement y PID: Kp=%f, Ki=%f, Kd=%f, limit=%f Nm\n", CTRL.PID_disy.Kp, CTRL.PID_disy.Ki,CTRL.PID_disy.Kd, CTRL.PID_disy.limit);
}

void circuit_main_A(){
    
    //  when the A phase is on
    if (CTRL.angle_A>CTRL.advance_angle-pi/24 && CTRL.angle_A<CTRL.advance_angle+pi/24){
     
        if(BSRM.IA<CTRL.main_current-CTRL.current_hystersis) {CTRL.UA=CTRL.Um;} //depend on the circuit topology
        else if(BSRM.IA>CTRL.main_current+CTRL.current_hystersis){CTRL.UA=-CTRL.Um;}
        else {}
        // the current could be positive or nagative
        if(BSRM.IA_X<CTRL.IA_X-CTRL.current_hystersis) {CTRL.UA_X=CTRL.Us;}
        else if(BSRM.IA_X>CTRL.IA_X+CTRL.current_hystersis){CTRL.UA_X=-CTRL.Us;}
        else {}

        if(BSRM.IA_Y<CTRL.IA_Y-CTRL.current_hystersis) {CTRL.UA_Y=CTRL.Us;}
        else if(BSRM.IA_X>CTRL.IA_X+CTRL.current_hystersis){CTRL.UA_Y=-CTRL.Us;}
        else {}

    }
    //when the A phase is off , the  freewheel-current  is consider.
    else{
        if(BSRM.IA>0){
            CTRL.UA=-CTRL.Um;
        }
        else if(BSRM.IA<=0) 
        {
            BSRM.IA=0;
            CTRL.UA=0;
        }
        
        if(BSRM.IA_X>0){
            if(sign(BSRM.IA_X) == sign(BSRM.IA_X_pre)){
                CTRL.UA_X=-CTRL.Us;
            }
            else{
                CTRL.UA_X=0;
                BSRM.IA_X=0;
            }
        }
        else if(BSRM.IA_X<0){
            if (sign(BSRM.IA_X) == sign(BSRM.IA_X_pre)){
                CTRL.UA_X=CTRL.Us;
            }
            else{
                CTRL.UA_X=0;
                BSRM.IA_X=0;
            }
        }
        else{
            CTRL.UA_X=0;
            BSRM.IA_X=0;
        }
        
        if(BSRM.IA_Y>0){
            if(sign(BSRM.IA_Y) == sign(BSRM.IA_Y_pre)){
                CTRL.UA_Y=-CTRL.Us;
            }
            else{
                CTRL.UA_Y=0;
                BSRM.IA_Y=0;
            }
        }
        else if(BSRM.IA_Y<0){
            if (sign(BSRM.IA_Y) == sign(BSRM.IA_Y_pre)){
                CTRL.UA_Y=CTRL.Us;
            }
            else{
                CTRL.UA_Y=0;
                BSRM.IA_Y=0;
            }
        }
        else{
            CTRL.UA_Y=0;
            BSRM.IA_Y=0;
        }
        
    }
    BSRM.IA_X_pre=BSRM.IA_X;
    BSRM.IA_Y_pre=BSRM.IA_Y; 
}

void circuit_main_B(){
    if (CTRL.angle_B>CTRL.advance_angle-pi/24 && CTRL.angle_B<CTRL.advance_angle+pi/24){

        if(BSRM.IB<CTRL.main_current-CTRL.current_hystersis) {CTRL.UB=CTRL.Um;} //depend on the circuit topology
        else if(BSRM.IB>CTRL.main_current+CTRL.current_hystersis){CTRL.UB=-CTRL.Um;}
        else {}

        if(BSRM.IB_X<CTRL.IB_X-CTRL.current_hystersis) {CTRL.UB_X=CTRL.Us;}
        else if(BSRM.IB_X>CTRL.IB_X+CTRL.current_hystersis){CTRL.UB_X=-CTRL.Us;}
        else {}

        if(BSRM.IB_Y<CTRL.IB_Y-CTRL.current_hystersis) {CTRL.UB_Y=CTRL.Us;}
        else if(BSRM.IB_X>CTRL.IB_X+CTRL.current_hystersis){CTRL.UB_Y=-CTRL.Us;}
        else {}
 
    }

    else{
        if(BSRM.IB>0){
            CTRL.UB=-CTRL.Um;
        }
        else if(BSRM.IB<=0) 
        {
            BSRM.IB=0;
            CTRL.UB=0;
        }
        
        if(BSRM.IB_X>0){
            if(sign(BSRM.IB_X) == sign(BSRM.IB_X_pre)){
                CTRL.UB_X=-CTRL.Us;
            }
            else{
                CTRL.UB_X=0;
                BSRM.IB_X=0;
            }
        }
        else if(BSRM.IB_X<0){
            if (sign(BSRM.IB_X) == sign(BSRM.IB_X_pre)){
                CTRL.UB_X=CTRL.Us;
            }
            else{
                CTRL.UB_X=0;
                BSRM.IB_X=0;
            }
        }
        else{
            CTRL.UB_X=0;
            BSRM.IB_X=0;
        }
        
        if(BSRM.IB_Y>0){
            if(sign(BSRM.IB_Y) == sign(BSRM.IB_Y_pre)){
                CTRL.UB_Y=-CTRL.Us;
            }
            else{
                CTRL.UB_Y=0;
                BSRM.IB_Y=0;
            }
        }
        else if(BSRM.IB_Y<0){
            if (sign(BSRM.IB_Y) == sign(BSRM.IB_Y_pre)){
                CTRL.UB_Y=CTRL.Us;
            }
            else{
                CTRL.UB_Y=0;
                BSRM.IB_Y=0;
            }
        }
        else{
            CTRL.UB_Y=0;
            BSRM.IB_Y=0;
        }

                
    }
    BSRM.IB_X_pre=BSRM.IB_X;
    BSRM.IB_Y_pre=BSRM.IB_Y;  
}

void circuit_main_C(){

    if (CTRL.angle_C>CTRL.advance_angle-pi/24 && CTRL.angle_C<CTRL.advance_angle+pi/24){

        if(BSRM.IC<CTRL.main_current-CTRL.current_hystersis) {CTRL.UC=CTRL.Um;} //depend on the circuit topology
        else if(BSRM.IC>CTRL.main_current+CTRL.current_hystersis){CTRL.UC=-CTRL.Um;}
        else {}

        if(BSRM.IC_X<CTRL.IC_X-CTRL.current_hystersis) {CTRL.UC_X=CTRL.Us;}
        else if(BSRM.IC_X>CTRL.IC_X+CTRL.current_hystersis){CTRL.UC_X=-CTRL.Us;}
        else {}

        if(BSRM.IC_Y<CTRL.IC_Y-CTRL.current_hystersis) {CTRL.UC_Y=CTRL.Us;}
        else if(BSRM.IC_X>CTRL.IC_X+CTRL.current_hystersis){CTRL.UC_Y=-CTRL.Us;}
        else {}
    
    }
    else{
        if(BSRM.IC>0){
            CTRL.UC=-CTRL.Um;
        }
        else if(BSRM.IC<=0) 
        {
            BSRM.IC=0;
            CTRL.UC=0;
        }
        
        
        if(BSRM.IC_X>0){
            if(sign(BSRM.IC_X) == sign(BSRM.IC_X_pre)){
                CTRL.UC_X=-CTRL.Us;
            }
            else{
                CTRL.UC_X=0;
                BSRM.IC_X=0;
            }
        }
        else if(BSRM.IC_X<0){
            if (sign(BSRM.IC_X) == sign(BSRM.IC_X_pre)){
                CTRL.UC_X=CTRL.Us;
            }
            else{
                CTRL.UC_X=0;
                BSRM.IC_X=0;
            }
        }
        else{
            CTRL.UC_X=0;
            BSRM.IC_X=0;
        }
        
        if(BSRM.IC_Y>0){
            if(sign(BSRM.IC_Y) == sign(BSRM.IC_Y_pre)){
                CTRL.UC_Y=-CTRL.Us;
            }
            else{
                CTRL.UC_Y=0;
                BSRM.IC_Y=0;
            }
        }
        else if(BSRM.IC_Y<0){
            if (sign(BSRM.IC_Y) == sign(BSRM.IC_Y_pre)){
                CTRL.UC_Y=CTRL.Us;
            }
            else{
                CTRL.UC_Y=0;
                BSRM.IC_Y=0;
            }
        }
        else{
            CTRL.UC_Y=0;
            BSRM.IC_Y=0;
        }
    }

    BSRM.IC_X_pre=BSRM.IC_X;
    BSRM.IC_Y_pre=BSRM.IC_Y;    
}

void control(int step){
    static int ctrl_judge=0;

    ctrl_judge++;
    if(ctrl_judge == NUMBER_OF_STEPS/CTRL_FRE){//
        ctrl_judge=0;
        CTRL.ctrl_count++;
        // Input 1 is feedback: measured speed
        CTRL.advance_angle=PID_Speed(&CTRL.PID_speed,CTRL.rpm-CTRL.rpm_cmd);//CTRL.rpm_cmd
        
        // Input 2 is feedback: measured current in x
        CTRL.x_force=PID_Dis(&CTRL.PID_disx,-CTRL.x_displacement*1000000);
        // Input 3  is feedback: measured current in y
        CTRL.y_force=PID_Dis(&CTRL.PID_disy,-CTRL.y_displacement*1000000);

        //force_axis_transfer
        CTRL.x_force_A=CTRL.x_force;
        CTRL.y_force_A=CTRL.y_force;
        CTRL.x_force_B=CTRL.x_force*sqrt3/2 + CTRL.y_force/2;
        CTRL.y_force_B= - CTRL.x_force/2 + CTRL.y_force*sqrt3/2;
        CTRL.x_force_C=CTRL.x_force*sqrt3/2 + CTRL.y_force/2;
        CTRL.y_force_C= - CTRL.x_force/2 +CTRL.y_force*sqrt3/2;

        if (CTRL.angle_A>CTRL.advance_angle-pi/24 && CTRL.angle_A<CTRL.advance_angle+pi/24){
            CTRL.IA_X=(CTRL.Kf1_A*CTRL.x_force_A-CTRL.Kf2_A*CTRL.y_force_A)/CTRL.main_current/(CTRL.Kf2_A*CTRL.Kf2_A+CTRL.Kf1_A*CTRL.Kf1_A);
            CTRL.IA_Y=(CTRL.Kf2_A*CTRL.x_force_A-CTRL.Kf1_A*CTRL.y_force_A)/CTRL.main_current/(CTRL.Kf2_A*CTRL.Kf2_A-CTRL.Kf1_A*CTRL.Kf1_A);
        }
        else{
            CTRL.IA_X=0;
            CTRL.IA_Y=0;
        }

        
        if (CTRL.angle_B>CTRL.advance_angle-pi/24 && CTRL.angle_B<CTRL.advance_angle+pi/24){
            CTRL.IB_X=(CTRL.Kf1_B*CTRL.x_force_B-CTRL.Kf2_B*CTRL.y_force_B)/CTRL.main_current/(CTRL.Kf2_B*CTRL.Kf2_B+CTRL.Kf1_B*CTRL.Kf1_B);
            CTRL.IB_Y=(CTRL.Kf2_B*CTRL.x_force_B-CTRL.Kf1_B*CTRL.y_force_B)/CTRL.main_current/(CTRL.Kf2_B*CTRL.Kf2_B-CTRL.Kf1_B*CTRL.Kf1_B);
        }
        else{
            CTRL.IB_X=0;
            CTRL.IB_Y=0;
        }

        if (CTRL.angle_C>CTRL.advance_angle-pi/24 && CTRL.angle_C<CTRL.advance_angle+pi/24){
            CTRL.IC_X=(CTRL.Kf1_C*CTRL.x_force_C-CTRL.Kf2_C*CTRL.y_force_C)/CTRL.main_current/(CTRL.Kf2_C*CTRL.Kf2_C+CTRL.Kf1_C*CTRL.Kf1_C);
            CTRL.IC_Y=(CTRL.Kf2_C*CTRL.x_force_C-CTRL.Kf1_C*CTRL.y_force_C)/CTRL.main_current/(CTRL.Kf2_C*CTRL.Kf2_C-CTRL.Kf1_C*CTRL.Kf1_C);
        }
        else{
            CTRL.IC_X=0;
            CTRL.IC_Y=0;
        }

        if (CTRL.IA_X>CTRL.MAX_DIS){CTRL.IA_X=CTRL.MAX_DIS;}
        else if(CTRL.IA_X<-CTRL.MAX_DIS){CTRL.IA_X=-CTRL.MAX_DIS;}
        if (CTRL.IA_Y>CTRL.MAX_DIS){CTRL.IA_Y=CTRL.MAX_DIS;}
        else if(CTRL.IA_Y<-CTRL.MAX_DIS){CTRL.IA_Y=-CTRL.MAX_DIS;}

        if (CTRL.IB_X>CTRL.MAX_DIS){CTRL.IB_X=CTRL.MAX_DIS;}
        else if(CTRL.IB_X<-CTRL.MAX_DIS){CTRL.IB_X=-CTRL.MAX_DIS;}
        if (CTRL.IB_Y>CTRL.MAX_DIS){CTRL.IB_Y=CTRL.MAX_DIS;}
        else if(CTRL.IB_Y<-CTRL.MAX_DIS){CTRL.IB_Y=-CTRL.MAX_DIS;}

        if (CTRL.IC_X>CTRL.MAX_DIS){CTRL.IC_X=CTRL.MAX_DIS;}
        else if(CTRL.IC_X<-CTRL.MAX_DIS){CTRL.IC_X=-CTRL.MAX_DIS;}
        if (CTRL.IC_Y>CTRL.MAX_DIS){CTRL.IC_Y=CTRL.MAX_DIS;}
        else if(CTRL.IC_Y<-CTRL.MAX_DIS){CTRL.IC_Y=-CTRL.MAX_DIS;}
//      BSRM.x_force_A=IA*(BSRM.Kf1_A*IA_X+BSRM.Kf2_A*IA_Y);
//      BSRM.y_force_A=IA*(-BSRM.Kf2_A*IA_X+BSRM.Kf1_A*IA_Y);

    }
    // #if #else #endif   is utilized to decide which part of codes is to be interpreted

    
}
