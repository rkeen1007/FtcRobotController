package org.firstinspires.ftc.teamcode.OpMode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.Utility.*;

@TeleOp(name = "TeleOp - chassis")
public class Chassis013026 extends LinearOpMode {

  private DcMotor FrontLeftWheel;
  private DcMotor FrontRightWheel;
  private DcMotor BackLeftWheel;
  private DcMotor BackRightWheel;
  private Servo RightTower;
  private Servo LeftTower;
  private Servo LauncherServo;
  private DcMotor LeftLaunchMotor;
  private DcMotor RightLaunchMotor;

  EyeSeeAprilTags eyeballtest = new EyeSeeAprilTags();
  int obeliskID = 0;
  
  @Override
  public void runOpMode() {
    double forward;
    double strafe;
    double rotation;
    double power = 1;
    int switchNumber = 0;
    
    double frontSpeedLauncher = -0.85;
    double backSpeedLauncher = -1.0;

    FrontLeftWheel = hardwareMap.get(DcMotor.class, "FrontLeftWheel");
    FrontRightWheel = hardwareMap.get(DcMotor.class, "FrontRightWheel");
    BackLeftWheel = hardwareMap.get(DcMotor.class, "BackLeftWheel");
    BackRightWheel = hardwareMap.get(DcMotor.class, "BackRightWheel");
    RightTower = hardwareMap.get(Servo.class, "RightTower");
    LeftTower = hardwareMap.get(Servo.class, "LeftTower");
    LauncherServo = hardwareMap.get(Servo.class, "LauncherServo");
    LeftLaunchMotor = hardwareMap.get(DcMotor.class, "LeftLaunchMotor");
    RightLaunchMotor = hardwareMap.get(DcMotor.class, "RightLaunchMotor");
    
    
    
    FrontLeftWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    FrontRightWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    BackLeftWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    BackRightWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    
    //this thread prevents lose of control of base
    Thread Chassis = new Thread(){
                         public void run(){
                           
                            if (opModeIsActive()) {
                            while (opModeIsActive()) {
                           
                           double forward;
                           double strafe;
                           double rotation;
   
        strafe = gamepad1.right_stick_x;
        forward = gamepad1.right_stick_y;
        rotation = gamepad1.left_stick_x;
        
        FrontLeftWheel.setPower(-(strafe - (forward - rotation)));
        FrontRightWheel.setPower(-(strafe + forward + rotation));
        BackLeftWheel.setPower(-(strafe + (forward - rotation)));
        BackRightWheel.setPower(-(strafe - (forward + rotation)));
        
        if(strafe == 0.0 && forward == 0.0 && rotation == 0.0){
          FrontLeftWheel.setPower(0);
          FrontRightWheel.setPower(0);
          BackLeftWheel.setPower(0);
          BackRightWheel.setPower(0);
        }
  
          }
        }
      }// end of Chassis run function
    }; //end of Chassis thread
    
    Thread webcam = new Thread(){
                         public void run(){
                      
                            if (opModeIsActive()) {
                            while (opModeIsActive()) {
                      
                         double[] position = eyeballtest.getCurrentPoseArray();
                           
                          position = eyeballtest.getCurrentPoseArray();
                          
                          int sideColor = (int)position[6];
                          telemetry.addLine("Attempting to read position tag");
                          telemetry.addData("POS ID", sideColor);
                          telemetry.update();

                           obeliskID = eyeballtest.telemetryAprilTag(telemetry);
                           telemetry.addLine("Attempting to read Obelisk");
                           telemetry.addData("Obelisk ID", obeliskID);
                           telemetry.update();
                            
                           
                        
        
  
          }
        }
      }// end of webcam run function
    }; //end of webcam thread

     

    eyeballtest.initAprilTag(hardwareMap);
    
    waitForStart();
    
    Chassis.start();
    webcam.start();
    
    if (opModeIsActive()) {
      while (opModeIsActive()) {
        
        /*
        strafe = gamepad1.right_stick_x;
        forward = gamepad1.right_stick_y;
        rotation = gamepad1.left_stick_x;
        
        FrontLeftWheel.setPower(-(strafe - (forward - rotation)));
        FrontRightWheel.setPower(-(strafe + forward + rotation));
        BackLeftWheel.setPower(-(strafe + (forward - rotation)));
        BackRightWheel.setPower(-(strafe - (forward + rotation)));
        
        if(strafe == 0.0 && forward == 0.0 && rotation == 0.0){
          FrontLeftWheel.setPower(0);
          FrontRightWheel.setPower(0);
          BackLeftWheel.setPower(0);
          BackRightWheel.setPower(0);
        }
        */
        
       if (gamepad2.right_bumper) {
          RightTower.setPosition(0);
          sleep(1000);
          //RightTower.setPosition(0.3);
          //sleep(1000);
          RightTower.setPosition(0.12);
        }
        
        
        if (gamepad2.left_bumper) {
          LeftTower.setPosition(0.3);
          sleep(1000);
          LeftTower.setPosition(0);
        }
        
        
        
        if (gamepad2.b) {
          LauncherServo.setPosition(0.2);
        }
        
        
        if (gamepad2.y) {
          LauncherServo.setPosition(0);
        }
        
        
        if (gamepad2.a) {
          LauncherServo.setPosition(0.08);
        }



    if((gamepad2.left_trigger > 0) && (LeftLaunchMotor.getPower() == 0)) {
      LeftLaunchMotor.setPower(backSpeedLauncher);
      sleep(500);
    }
    
    
    if((gamepad2.left_trigger > 0) && (LeftLaunchMotor.getPower() != 0)) {
      LeftLaunchMotor.setPower(0);
      sleep(500);
    }
    

    
    if((gamepad2.right_trigger > 0) && (LeftLaunchMotor.getPower() == 0)) {
      LeftLaunchMotor.setPower(frontSpeedLauncher);
      sleep(500);
    }
    
    
    if((gamepad2.right_trigger > 0) && (LeftLaunchMotor.getPower() != 0)) {
      LeftLaunchMotor.setPower(0);
      sleep(500);
    }


       
        
        
      }
    }
  }
}
