# Dymanic_simulation_BSRM
 

### THIS IS A FRAMEWORK FOR BSRM DYNEMIC SIMULATION,MOREOVER, ALSO SUITABLE FOR SRM SIMULATION IF THE CHRRENT OF SUSPENSION WINDINGS IS SET ZERO IN EXTERN FUNCTION NAMED CIRCUIT() IN CONTROL.C


# in main.c function measurement_quidk(), add the following codes to reset framework for convetional SRM simulation
{
    BSRM.IA_X=0;
    BSRM.IA_Y=0;
    BSRM.IB_X=0;
    BSRM.IB_Y=0;
    BSRM.IC_X=0;
    BSRM.IC_Y=0;
}


