package org.firstinspires.ftc.teamcode.OpMode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import java.util.Arrays;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


import org.firstinspires.ftc.teamcode.Utility.*;



@Autonomous(name = "Auto - Back Wall BLUE Start")



public class BackWall_BLUE_StartAuto extends LinearOpMode {

    //instantiate utility class
    DriveTrain drive = new DriveTrain();
    ElapsedTime runtime = new ElapsedTime();
    Launcher launch = new Launcher();
    
    
    
     //time in seconds
     double endTime = 1.0;
     //keeps main while loop from running more than once
     int loopCount = 0;
     
     double forwardSpeed = 0.35;
     double strafeSpeed = -0.75;
     
     int sleepTimer = 1000;
 
    double launchPower = 1.0;

    int obeliskID = 0;

    
    @Override
    public void runOpMode() {
        
        // run the init methods
        drive.initDriveTrain(hardwareMap);
        launch.initLaunch(hardwareMap);

    
        waitForStart();
              
        
        while (opModeIsActive() && (loopCount <1)) {
            
            //sets to 2 so that while loop will not repeat
            loopCount = 2;
            
            runtime.reset();
            // Drive Backwards to launch position
            while(runtime.seconds() <= (endTime/2)){
                                
                //move(forward, strafe, rotation)
                drive.move(0.0,strafeSpeed,0.0);
                
            }//end while
            drive.brakes();
          
          
          
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
          
          
          
            
            runtime.reset();
            // Drive Backwards to launch position
            while(runtime.seconds() <= (endTime/2)){
                                
                //move(forward, strafe, rotation)
                drive.move(forwardSpeed,0.0,0.0);
                
            }//end while
            drive.brakes();
          
            sleep(sleepTimer);  
            
           
    
            
            
                
        }//end while OpmodeActive

    }   // end method runOpMode()

}   // end class
