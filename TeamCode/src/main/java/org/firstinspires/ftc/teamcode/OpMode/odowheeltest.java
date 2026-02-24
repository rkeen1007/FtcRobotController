package org.firstinspires.ftc.teamcode.OpMode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import org.firstinspires.ftc.teamcode.Utility.*;



@TeleOp(name = "odo wheeltest")
public class odowheeltest extends LinearOpMode {

    GoBildaPinpointDriver odocomp;

    double currentX;
    double currentY;
    double currentHeading;

@Override
    public void runOpMode() {

        //initialize variables here
        odocomp = hardwareMap.get(GoBildaPinpointDriver.class, "odocomp");
       

        

        
        //The X pod offset refers to how far sideways (in mm) from the tracking point the X (forward)
        //Left of the center is a positive number, right of center is a negative number. the Y pod offset refers to how far forwards (in mm) from the tracking point the Y
        // xOffset, yOffset
        odocomp.setOffsets(-0.11811,1.06299, DistanceUnit.INCH);
        //sets the type of odometry wheels
        odocomp.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        //set direction of encorders
        odocomp.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                                        GoBildaPinpointDriver.EncoderDirection.FORWARD);


        odocomp.resetPosAndIMU();




        

        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                
                
            
            //super critical only run once, then delete this line
            if(gamepad1.back){
                odocomp.recalibrateIMU();
            }//end of if needs deleted after calibrate
        
        

            odocomp.update();
            currentX = odocomp.getPosX(DistanceUnit.INCH);
            currentY = odocomp.getPosY(DistanceUnit.INCH);
            currentHeading = odocomp.getHeading(AngleUnit.DEGREES);
            

            telemetry.addData("currentX",currentX);
            telemetry.addData("currentY", currentY);
            telemetry.addData("currentHeading",currentHeading);
            telemetry.update();
        
            }//end while active loop
        }//end if active
    }//end runopmode


 



}//end class