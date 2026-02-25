package org.firstinspires.ftc.teamcode.z_TestCodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.List;
import java.util.ArrayList;

//odometry wheel dependent libraries
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

//Lake Robotics Utility folder
import org.firstinspires.ftc.teamcode.Utility.*;

//set driver hub name for this opmode
@TeleOp(name = "Keen_Test_22626")



public class Keen_Test_22626 extends LinearOpMode {

    //   DEFINE  VARIABLES


    //measured values for initial position of robot settings
    // Check and re-assign as needed based on starting location
    double X_i = 0.0;
    double Y_i = 0.0;
    double Yaw_i = 90.0;

    // {x,y,z,pitch,roll,yaw,ID}
    double[] position = {X_i,Y_i,0.0,0.0,0.0,Yaw_i,21};
    // position coordinates
    double currentX;
    double currentY;
    double currentHeading;


    // pre-measured target location {x, y, yaw_to_launch} relative to field
    //target position = {x_location, y_location, desired_arenacoord_angle_for_launch}
    double[] targetPos = {25.0,25.0,45.0};

    //targetVec is used to store the dx,dy,phi, magnitudetoTarget, AngletoTurn
    double[] targetVec;


    // pre-measured acceptable error between actual-target
    //degrees
    double errorAngle = 5.0;
    //inches
    double errorDistance = 2.0;

    //a true/false for angle and distance movements
    boolean correctAngle = false;
    boolean correctDistance = false;
    boolean[] onTarget = {correctAngle,correctDistance};

    List<Double> valuesPID = new ArrayList<>();
    double pidValue;


    //odometry computer variable setup
    GoBildaPinpointDriver odocomp;











    //   INSTANTIATE OBJECTS
    //instantiate utility class


    //motor controls
    DriveTrain drive = new DriveTrain();

    //timer function
    // ElapsedTime runtime = new ElapsedTime()
    // PID calculator - parameters = kp, KI, KD, ClipValue
    PIDController pid = new PIDController(0.01,0.01,0.01, 0.50);
    //2d position array fro initial setting of our odometry computer
    Pose2D initialPos = new Pose2D(DistanceUnit.INCH,X_i,Y_i,AngleUnit.DEGREES,Yaw_i);
    //calculation of vectors (currently 2/24/26 only difference vector targetVec variable)
    Calculations calc = new Calculations();






    @Override
    public void runOpMode() {



// RUN INIT METHODS FOR OBJECTS

        drive.initDriveTrain(hardwareMap);

        odocomp = hardwareMap.get(GoBildaPinpointDriver.class, "odocomp");
        //objectDetect.initSensor(hardwareMap);





        //    ODOMETRY COMPUTER SETUP
        //The X pod offset refers to how far sideways (in mm) from the tracking point the X (forward)
        //Left of the center is a positive number, right of center is a negative number. the Y pod offset refers to how far forwards (in mm) from the tracking point the Y
        // xOffset, yOffset
        odocomp.setOffsets(-3.732,1.327, DistanceUnit.INCH);
        //sets the type of odometry wheels
        odocomp.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        //set direction of encorders
        odocomp.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);

        odocomp.setPosition(initialPos);




        waitForStart();



        while (opModeIsActive()) {



            odocomp.update();
            currentX = odocomp.getPosX(DistanceUnit.INCH);
            currentY = odocomp.getPosY(DistanceUnit.INCH);
            currentHeading = odocomp.getHeading(AngleUnit.DEGREES);


            //assign current position to position array
            position[0] = currentX;
            position[1] = currentY;
            position[5] = currentHeading;


            //update the targetVec with dx,dy,phi,magnitude,turnAngle
            targetVec = calc.calcDiffVec(position,targetPos);




            telemetry.addLine("OUTSIDE While Loop");
            telemetry.addData("currentX",currentX);
            telemetry.addData("currentY", currentY);
            telemetry.addData("currentHeading",currentHeading);
            telemetry.addData("Turn Angle", targetVec[4]);
            telemetry.addData("difference", (position[5]-targetVec[4]));
            telemetry.update();


        }


    }
}
