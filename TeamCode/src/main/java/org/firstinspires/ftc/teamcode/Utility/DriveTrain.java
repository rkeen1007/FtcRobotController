package org.firstinspires.ftc.teamcode.Utility;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DriveTrain {

  private DcMotor FrontLeftWheel;
  private DcMotor FrontRightWheel;
  private DcMotor BackLeftWheel;
  private DcMotor BackRightWheel;



  public void initDriveTrain(HardwareMap hardwareMap){
    
    FrontLeftWheel = hardwareMap.get(DcMotor.class, "FrontLeftWheel");
    FrontRightWheel = hardwareMap.get(DcMotor.class, "FrontRightWheel");
    BackLeftWheel = hardwareMap.get(DcMotor.class, "BackLeftWheel");
    BackRightWheel = hardwareMap.get(DcMotor.class, "BackRightWheel");
    
    FrontLeftWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    FrontRightWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    BackLeftWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    BackRightWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    
  }//end init


  public void move(double Forward, double Strafe, double Rotation){
    
            
    double forward = -1*Forward;
    double strafe = -1*Strafe;
    double rotation = -1*Rotation;

    FrontLeftWheel.setPower(-(strafe - (forward - rotation)));
    FrontRightWheel.setPower(-(strafe + forward + rotation));
    BackLeftWheel.setPower(-(strafe + (forward - rotation)));
    BackRightWheel.setPower(-(strafe - (forward + rotation)));
  
  }//end move
  
  public void brakes(){
    FrontLeftWheel.setPower(0);
    FrontRightWheel.setPower(0);
    BackLeftWheel.setPower(0);
    BackRightWheel.setPower(0);
  }// end stop
  
  
  
 /*
  public void collisionAvoid(boolean[] detectedObjects){
      
      
      //order of sensors [BS, FS, RS, LS]
      boolean[] objectsDetected = detectedObjects;
      
      //back
      if (objectsDetected[0] == false){
            FrontLeftWheel.setPower(-(strafe - (forward - rotation)));
            FrontRightWheel.setPower(-(strafe + forward + rotation));
            BackLeftWheel.setPower(-(strafe + (forward - rotation)));
            BackRightWheel.setPower(-(strafe - (forward + rotation)));
          
      }//end back
      
      //right
      if (objectsDetected[2] == false){
          
      }//end right
      
      //left
      if (objectsDetected[3] == false){
          
      }//end left
      
      
      
      
      
  }//end avoid

 */
 
}// end class