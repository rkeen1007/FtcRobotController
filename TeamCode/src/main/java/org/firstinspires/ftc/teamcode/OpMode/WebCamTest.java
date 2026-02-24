package org.firstinspires.ftc.teamcode.OpMode;



import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import java.util.Arrays;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Utility.*;

@TeleOp(name = "WebCam Test")
public class WebCamTest extends LinearOpMode {

    //instantiate utility class
    EyeSeeAprilTags eyeball = new EyeSeeAprilTags();
    ElapsedTime runtime = new ElapsedTime();
    //IRSensor objectDetect = new IRSensor();
    

    @Override
    public void runOpMode() {
        
        // run the init methods
        eyeball.initAprilTag(hardwareMap);

        
    
        waitForStart();
    if (opModeIsActive()) {
      while (opModeIsActive()) {
             //read april tag on goals
            double[] position = eyeball.getCurrentPoseArray();
            position = eyeball.getCurrentPoseArray();
            
            telemetry.addData("x", position[0]);
            telemetry.addData("y", position[1]);
            telemetry.addData("angle", position[5]);
            telemetry.update();
            
        }//end while OpmodeActive
    }//end if active
    }   // end method runOpMode()

}   // end class
