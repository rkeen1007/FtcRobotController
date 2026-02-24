// package org.firstinspires.ftc.teamcode.OpMode;
// 
// //auto opmode libraries
// import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
// import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
// 
// //other libraries
// import java.util.Arrays;
// import java.util.List;
// import java.util.ArrayList;
// import com.qualcomm.robotcore.util.ElapsedTime;
// 
// //odometry wheel dependent libraries
// import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
// import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
// import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
// import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
// 
// //Lake Robotics Utility folder
// import org.firstinspires.ftc.teamcode.Utility.*;
// 
// 
// 
// //set driver hub name for this opmode
// @Autonomous(name = "test21226")
// 
// 
// 
// public class testcode21226 extends LinearOpMode {
// 
// 
// 
// 
// //   DEFINE  VARIABLES  
// 
//     List<Double> valuesPID = new ArrayList<>();
//     
//     
//     // {x,y,z,pitch,roll,yaw,ID}
//     double[] position = {0.0,0.0,0.0,0.0,0.0,0.0,21};
//     // position coordinates
//     double currentX;
//     double currentY;
//     double currentHeading;
//     
//     
//     //measured values for initial position of robot settings
//     // Check and re-assign as needed based on starting location
//     double X_i = 0.0;
//     double Y_i = 0.0;
//     double Yaw_i = 90.0;
// 
// 
//     // pre-measured target location {x, y, yaw} relative to field
// 
//     double[] targetPos = {0.0,0.0,45.0};
// 
// 
//     // pre-measured acceptable error between actual-target
//     //degrees
//     double errorAngle = 5.0;
//     //inches
//     double errorDistance = 2.0;
//      
//     double pidValue;
// 
// 
//     //odometry computer variable setup
//     GoBildaPinpointDriver odocomp;
// 
// 
// 
// 
// 
// 
// 
// 
// 
// 
// 
//     //   INSTANTIATE OBJECTS
//     //instantiate utility class
//     EyeSeeAprilTags eyeball = new EyeSeeAprilTags();
//     DriveTrain drive = new DriveTrain();
//     Launcher launch = new Launcher();
//     ElapsedTime runtime = new ElapsedTime();
//     PIDController pid = new PIDController(0.01,0.01,0.01);
//     Pose2D initialpos = new Pose2D(DistanceUnit.INCH,X_i,Y_i,AngleUnit.DEGREES,Yaw_i);
//     //IRSensor objectDetect = new IRSensor();
// 
// 
// 
// 
// 
//    
//    
// 
// 
// 
// 
// 
// 
// 
//    
// 
//     @Override
//     public void runOpMode() {
//        
// 
// 
//         // RUN INIT METHODS FOR OBJECTS
//         eyeball.initAprilTag(hardwareMap);
//         drive.initDriveTrain(hardwareMap);
//         launch.initLaunch(hardwareMap);
//         odocomp = hardwareMap.get(GoBildaPinpointDriver.class, "odocomp");
// 
// 
//         //objectDetect.initSensor(hardwareMap);
//        
// 
// 
// 
// 
// 
// 
// 
//         //    ODOMETRY COMPUTER SETUP
//         //The X pod offset refers to how far sideways (in mm) from the tracking point the X (forward)
//         //Left of the center is a positive number, right of center is a negative number. the Y pod offset refers to how far forwards (in mm) from the tracking point the Y
//         // xOffset, yOffset
//         odocomp.setOffsets(-3.732,1.327, DistanceUnit.INCH);
//         //sets the type of odometry wheels
//         odocomp.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
// 
//         //set direction of encorders
//         odocomp.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
//                                         GoBildaPinpointDriver.EncoderDirection.FORWARD);
// 
// 
// 
//         odocomp.setPosition(initialpos);
// 
// 
// 
//    
//         waitForStart();
//              
//        
//         while (opModeIsActive()) {
//            
//         
// 
//             odocomp.update();
//             currentX = odocomp.getPosX(DistanceUnit.INCH);
//             currentY = odocomp.getPosY(DistanceUnit.INCH);
//             currentHeading = odocomp.getHeading(AngleUnit.DEGREES);
// 
// 
//             //read april tag on goals
//         //    position = eyeball.getCurrentPoseArray();
// 
// /*
//             // if the camera detects april tags on the goals, use them to set current position.
//             if(position[6] == 20 || position[6] == 24){
//                 currentX = position[0];
//                 currentY = position[1];
//                 currentHeading = position[5];
//             }//end if detects april tags
// */
//             //assign current position to position array
//             position[0] = currentX;
//             position[1] = currentY;
//             position[5] = currentHeading;
//         
//             telemetry.addLine("OUTSIDE While Loop");
//             telemetry.addData("difference", (position[5]-targetPos[2]));
//             telemetry.addData("currentX",currentX);
//             telemetry.addData("currentY", currentY);
//             telemetry.addData("currentHeading",currentHeading);
//             telemetry.addData("target yaw", targetPos[2]);
//             telemetry.addData("PID",pidValue);
//             telemetry.update();
// 
// 
// 
// 
// 
//             //turn until yaw equals phi within headingError
// 
//             // if yaw-phi greater than errorAngle
//             while( Math.abs((position[5]-targetPos[2])) > errorAngle){
// 
//                 
//                 // PIDController differenceYawPhi
//                 pidValue = pid.calculatePID(position[5],targetPos[2]);
//                 
//                 valuesPID.add(pidValue);
//                 // set motor rotate PID value
//                 //move(forward, strafe, rotation)
//                 drive.move(0.0,0.0,pidValue);
// 
//                 odocomp.update();
//                 currentX = odocomp.getPosX(DistanceUnit.INCH);
//                 currentY = odocomp.getPosY(DistanceUnit.INCH);
//                 currentHeading = odocomp.getHeading(AngleUnit.DEGREES);
//                 position[0] = currentX;
//                 position[1] = currentY;
//                 position[5] = currentHeading;
// 
// 
// 
//                 telemetry.addLine("INSIDE While Loop");
//                 telemetry.addData("difference", (position[5]-targetPos[2]));
//                 telemetry.addData("currentX",currentX);
//                 telemetry.addData("currentY", currentY);
//                 telemetry.addData("currentHeading",currentHeading);
//                 telemetry.addData("target yaw", targetPos[2]);
//                 telemetry.addData("PID",pidValue);
//                 telemetry.addData("listof PID", valuesPID);
//                 telemetry.update();
//                 
// 
// 
//             }//end while > errorAngle
//             
//             //reset PID to zero
//             pid.resetPID();
//             valuesPID.clear();
// 
//             drive.move(0.0,0.0,0.0);
//             drive.brakes();
// 
// 
// 
// 
// 
// 
//         }//end while OpmodeActive
// 
//     }   // end method runOpMode()
// 
// }   // end class
// 
// 
// 
// 
// 
// 
// 
// 
// 
// 
// 
// 
// 
// 