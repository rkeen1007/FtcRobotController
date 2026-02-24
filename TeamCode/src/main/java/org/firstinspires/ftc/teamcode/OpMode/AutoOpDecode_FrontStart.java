package org.firstinspires.ftc.teamcode.OpMode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import java.util.Arrays;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


import org.firstinspires.ftc.teamcode.Utility.*;



@Autonomous(name = "AutoOp - Front Start")



public class AutoOpDecode_FrontStart extends LinearOpMode {

    //instantiate utility class
    EyeSeeAprilTags eyeball = new EyeSeeAprilTags();
    DriveTrain drive = new DriveTrain();
    Launcher launch = new Launcher();
    ElapsedTime runtime = new ElapsedTime();
    //IRSensor objectDetect = new IRSensor();
    
    boolean[] detectedObstacle;
    
    
     //time in seconds
     double endTime = 1.0;
     //keeps main while loop from running more than once
     int loopCount = 0;
     //obelisk ID set to a default number for void case 
     int obeliskID = 0;
     
     double turnValue = 0.25;
     
     int sleepTimer = 1000;
     
     double launchPower = 0.85;


    
    @Override
    public void runOpMode() {
        
        // run the init methods
        eyeball.initAprilTag(hardwareMap);
        drive.initDriveTrain(hardwareMap);
        launch.initLaunch(hardwareMap);
        //objectDetect.initSensor(hardwareMap);
        
    
        waitForStart();
              
        
        while (opModeIsActive() && (loopCount <1)) {
            
            //sets to 2 so that while loop will not repeat
            loopCount = 2;
            
             //read april tag on goals
            double[] position = eyeball.getCurrentPoseArray();
            
            
            runtime.reset();
            // Drive Backwards to launch position
            while(runtime.seconds() <= endTime){
                
                
                //move(forward, strafe, rotation)
                drive.move(-0.50,0.0,0.0);
                 //read april tag on goals
                position = eyeball.getCurrentPoseArray();
            }//end while
            drive.brakes();
          
            sleep(sleepTimer);  
            
            //read april tag on goals
            position = eyeball.getCurrentPoseArray();

            //assigns 'sideColor' to the tagID in position[6]
            int sideColor = (int)position[6];
            telemetry.addLine("Attempting to read position tag");
            telemetry.addData("POS ID", sideColor);
            telemetry.update();

            sleep(sleepTimer);
            
            //turn towards obelisk
            //blue side
            if(sideColor == 20){
                telemetry.addLine("Blue Side Turn");
                telemetry.update();
                runtime.reset();
                while(runtime.seconds() <= (endTime/2)){
                    //move(forward, strafe, rotation)
                    drive.move(0.0,0.0,-turnValue);
                    obeliskID = eyeball.telemetryAprilTag(telemetry);
                }//end while
                drive.brakes();
            }//end if blue
            
            //red side
            if(sideColor == 24){
                telemetry.addLine("Red Side Turn");
                telemetry.update();
                runtime.reset();
                while(runtime.seconds() <= (endTime/2)){
                    //move(forward, strafe, rotation)
                    drive.move(0.0,0.0,turnValue);
                    obeliskID = eyeball.telemetryAprilTag(telemetry);
                }//end while
                drive.brakes();
            }//end if red


            
            //Read April Tag, determine pattern
            obeliskID = eyeball.telemetryAprilTag(telemetry);
            telemetry.addLine("Attempting to read Obelisk");
            telemetry.addData("Obelisk ID", obeliskID);
            telemetry.update();

            sleep(sleepTimer);

            //Turn back to goal
            //blue side
            if(sideColor == 20){
                telemetry.addLine("Blue Side Turn 2");
                telemetry.update();
                runtime.reset();
                while(runtime.seconds() <= (endTime/2)){
                    //move(forward, strafe, rotation)
                    drive.move(0.0,0.0,turnValue);
                    obeliskID = eyeball.telemetryAprilTag(telemetry);
                }//end while
                drive.brakes();
            }//end if blue
            
            //red side
            if(sideColor == 24){
                telemetry.addLine("Red Side Turn 2");
                telemetry.update();
                runtime.reset();
                while(runtime.seconds() <= (endTime/2)){
                    //move(forward, strafe, rotation)
                    drive.move(0.0,0.0,-turnValue);
                    obeliskID = eyeball.telemetryAprilTag(telemetry);
                }//end while
                drive.brakes();
            }//end if red


            sleep(sleepTimer);
            
            //Launch artifacts
            //GPP
            if(obeliskID == 21){
                    telemetry.addLine("GPP");
                    telemetry.update();
                    launch.spinLaunch(launchPower);
                    sleep(2000);
                    launch.greenLaunch();
                    sleep(1000);
                    launch.purpleLaunch();
                    sleep(1000);
                    launch.purpleLaunch();
                    sleep(1000);
                    launch.stopLaunch();
            }//end if GPP
            
            //PGP
            if(obeliskID == 22){
                    telemetry.addLine("PGP");
                    telemetry.update();
                    launch.spinLaunch(launchPower);
                    sleep(2000);
                    launch.purpleLaunch();
                    sleep(1000);
                    launch.greenLaunch();
                    sleep(1000);
                    launch.purpleLaunch();
                    sleep(1000);
                    launch.stopLaunch();
            }//end if PGP
            
            //PPG
            if(obeliskID == 23){
                    telemetry.addLine("PPG");
                    telemetry.update();
                    launch.spinLaunch(launchPower);
                    sleep(2000);
                    launch.purpleLaunch();
                    sleep(1000);
                    launch.purpleLaunch();
                    sleep(1000);
                    launch.greenLaunch();
                    sleep(1000);
                    launch.stopLaunch();
            }//end if PPG
            
            //if no launch order detected
            if(obeliskID == 0){
                    telemetry.addLine("Not Detected");
                    telemetry.update();
                    launch.spinLaunch(launchPower);
                    sleep(2000);
                    launch.greenLaunch();
                    sleep(1000);
                    launch.purpleLaunch();
                    sleep(1000);
                    launch.purpleLaunch();
                    sleep(1000);
                    launch.stopLaunch();
            }//end if no launch order detected

           
            sleep(sleepTimer);
            
                    
            // Drive OFF of launch line
            // blue side
            if(sideColor == 20){
                telemetry.addLine("Blue Side Strafe");
                telemetry.update();
                runtime.reset();
                while(runtime.seconds() <= (endTime/2)){
                    //move(forward, strafe, rotation)
                    drive.move(0.0,0.5,0.0);
                }//end while
                drive.brakes();
            }//end if blue
            
            //red side
            if(sideColor == 24){
                telemetry.addLine("Red Side Strafe");
                telemetry.update();
                runtime.reset();
                while(runtime.seconds() <= (endTime/2)){
                    //move(forward, strafe, rotation)
                    drive.move(0.0,-0.5,0.0);
                }//end while
                drive.brakes();
            }//end if red
    
            
            
                
        }//end while OpmodeActive

    }   // end method runOpMode()

}   // end class
