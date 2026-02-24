package org.firstinspires.ftc.teamcode.Utility;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;



public class Launcher{
  
  private Servo RightTower;
  private Servo LeftTower;
  private Servo LauncherServo;
  private DcMotor LeftLaunchMotor;
  private DcMotor RightLaunchMotor;
  
  
  public void initLaunch(HardwareMap hardwareMap){
    RightTower = hardwareMap.get(Servo.class, "RightTower");
    LeftTower = hardwareMap.get(Servo.class, "LeftTower");
    LauncherServo = hardwareMap.get(Servo.class, "LauncherServo");
    LeftLaunchMotor = hardwareMap.get(DcMotor.class, "LeftLaunchMotor");
    RightLaunchMotor = hardwareMap.get(DcMotor.class, "RightLaunchMotor");
    
    RightTower.setPosition(0.12);
    LeftTower.setPosition(0);
    
  }
  
  
  public void spinLaunch(double motorPower){
    
        LeftLaunchMotor.setPower(-1*motorPower);
        RightLaunchMotor.setPower(motorPower);
    
  }
  
  
  
  public void stopLaunch(){
        LeftLaunchMotor.setPower(0);
        RightLaunchMotor.setPower(0);
  }
  
  
  public void greenLaunch(){
          LauncherServo.setPosition(0.08);
          LeftTower.setPosition(0.3);
          
          try {
            Thread.sleep(1000);
          } //end try
          catch (InterruptedException e) {
            Thread.currentThread().interrupt();
          }//end catch
          
          LeftTower.setPosition(0);
          
          try {
            Thread.sleep(1000);
          } //end try
          catch (InterruptedException e) {
            Thread.currentThread().interrupt();
          }//end catch
          
          LauncherServo.setPosition(0.2);
          
          try {
            Thread.sleep(1000);
          } //end try
          catch (InterruptedException e) {
            Thread.currentThread().interrupt();
          }//end catch
          
          LauncherServo.setPosition(0.08);
          
  }
  
  public void purpleLaunch(){
          LauncherServo.setPosition(0.08);
          RightTower.setPosition(0);
          
          try {
            Thread.sleep(1000);
          } //end try
          catch (InterruptedException e) {
            Thread.currentThread().interrupt();
          }//end catch
          
          
          /*
          RightTower.setPosition(0.3);
          
          try {
            Thread.sleep(1000);
          } //end try
          catch (InterruptedException e) {
            Thread.currentThread().interrupt();
          }//end catch
          
          */
          
          RightTower.setPosition(0.12);
          
          try {
            Thread.sleep(1000);
          } //end try
          catch (InterruptedException e) {
            Thread.currentThread().interrupt();
          }//end catch
          
          LauncherServo.setPosition(0.2);
          
          try {
            Thread.sleep(1000);
          } //end try
          catch (InterruptedException e) {
            Thread.currentThread().interrupt();
          }//end catch
          
          LauncherServo.setPosition(0.08);
  }
    
    
}//end class


