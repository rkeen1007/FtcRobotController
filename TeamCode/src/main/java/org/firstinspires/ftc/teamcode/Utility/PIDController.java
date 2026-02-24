package org.firstinspires.ftc.teamcode.Utility;


import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;




public class PIDController{

    
    
    
    //How to ensure this reports in nanoseconds??
    ElapsedTime pidTimer = new ElapsedTime();
    
    
    
    public double deltaTime;
    public double positionError;
    public double outputPID;
    
    
    public double kP;
    public double kI;
    public double kD;
    public double pClip;
    
    
    public double integral = 0;    
    public double derivative = 0;
    
    public double lastError = 0;


    

    //sets the coefficients, same name means constructor of class (automatic)
    public PIDController(double kPset, double kIset, double kDset, double pidClip){
    
        kP = kPset;
        kI = kIset;
        kD = kDset;
        pClip = pidClip;
        
        pidTimer.reset();
        
    } //end setCoefPID method
    
    
        
        
        
    
    public double calculatePID(double current, double target){
                
        
        // find time since pidTimer was started
        deltaTime = pidTimer.nanoseconds();
        
        //reset timer to zero
        pidTimer.reset();
        
        
        
        //calculate the resultant vector difference magnitude in current vs target
        
        positionError =  target - current;
                
        
        // technically a rieman sum?
        integral += positionError * deltaTime;
        
        // rate of change since last error 
        derivative = (positionError - lastError) / deltaTime;
        
        
        // determine the value of the PID Controller output
        outputPID = (kP * positionError) + (kI * integral) + (kD * derivative);
        
        
        // store the current positionerror magnitude as the lasterror for next use
        lastError = positionError;
        
        outputPID = Range.clip(outputPID,-pClip,pClip);
        
        return outputPID;
    
    } // end calculatePID method
        
    
    
    //reset all values to zero for PID controller
    public void resetPID(){
        
        integral = 0;
        lastError = 0;
        pidTimer.reset();
        
    } //end resetPid method

}//end class

