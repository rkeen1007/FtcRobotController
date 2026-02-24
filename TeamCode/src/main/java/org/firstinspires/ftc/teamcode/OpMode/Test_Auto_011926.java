// package org.firstinspires.ftc.teamcode.OpMode;
// 
// //auto opmode libraries
// import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
// import org.firstinspires.ftc.robotcore.external.Telemetry;
// import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
// 
// //other libraries
// import java.util.Arrays;
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
// @Autonomous(name = "test 011926")
// 
// 
// 
// public class Test_Auto_011926 extends LinearOpMode {
// 
// 
//     //measured values for initial position of robot settings
//     // Check and re-assign as needed based on starting location
//     double X_i = 0.0;
//     double Y_i = 0.0;
//     double Yaw_i = 0.0;
// 
//     //   INSTANTIATE OBJECTS
//     //instantiate utility class
//     EyeSeeAprilTags eyeball = new EyeSeeAprilTags();
//     DriveTrain drive = new DriveTrain();
//     Launcher launch = new Launcher();
//     ElapsedTime runtime = new ElapsedTime();
//     PIDController pid = new PIDController(0.0,0.0,0.0);
//     
//     Pose2D initialpos = new Pose2D(DistanceUnit.INCH,X_i,Y_i,AngleUnit.DEGREES,Yaw_i);
//     
//     //IRSensor objectDetect = new IRSensor();
//     
//     
//     
//     
//     //   DEFINE  VARIABLES   
//     boolean[] detectedObstacle;
//     
//     
//     
//     // {dx,dy,magnitudeTarget,phi}
//     double dx = 10.0;
//     double dy = 10.0;
//     double magTarget = 14.1;
//     double phi = 45.0;
//     double[] targetVector = {dx,dy,magTarget,phi};
// 
//     // {x,y,z,pitch,roll,yaw,ID}
//     double[] position={X_i,Y_i,0.0,0.0,0.0,Yaw_i};
//     
//      //obelisk ID set to a default number to prevent void case 
//     int obeliskID = 0;
//      
//     
//     //odometry computer variable setup
//     GoBildaPinpointDriver odocomp;
// 
//     // position coordinates
//     double currentX;
//     double currentY;
//     double currentHeading;
//     
//     
//     
//     // pre-measured target location {x, y, yaw} relative to field
//     
//     double[] backTarget = {0.0,0.0,45.0};
//     double[] frontTarget = {0.0,0.0,45.0};
//     
//     //blank array to set the target in opmode
//     double[] chosenTarget = {0.0,0.0,0.0};
//     
//     // target distance from robot
//     double distanceFront;
//     double distanceBack;
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
//     //setting variables for initial position movement loop in opmode
//     boolean initialPositionDetected = false;
//     double endTime = 1.0;
//     double forwardPower = 0.5;
//     double rotatePower = 0.25; 
//     
//     
//     @Override
//     public void runOpMode() {
//         
//         
//         
//         //     RUN INIT METHODS FOR OBJECTS
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
//         //    ODOMETRY COMPUTER SETUP 
//         //The X pod offset refers to how far sideways (in mm) from the tracking point the X (forward)
//         //Left of the center is a positive number, right of center is a negative number. the Y pod offset refers to how far forwards (in mm) from the tracking point the Y
//         // xOffset, yOffset
//         odocomp.setOffsets(-0.11811,1.06299, DistanceUnit.INCH);
//         //sets the type of odometry wheels
//         odocomp.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
// 
//         //set direction of encorders
//         odocomp.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
//                                         GoBildaPinpointDriver.EncoderDirection.FORWARD);
// 
// 
//         odocomp.resetPosAndIMU();
//         
//         
//         odocomp.setPosition(initialpos);
//         
//         
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
//              //read april tag on goals
//             position = eyeball.getCurrentPoseArray();
//             
//             
//             //read obelisk only if not already identified
//             if(obeliskID == 0){
//                 //attempt to read obelisk
//                 obeliskID = eyeball.telemetryAprilTag(telemetry);
//             }//end read obelisk
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
//             /*// only runs one time in an attempt to locate an april tag to set our initial position
//             // ------ How do we add a check to see if it actually sets position?? May need to override all movement if not ----
//             if(initialPositionDetected == false){
//                 
//                 //avoid obstacles?
//                 
//                 
//                 // if first starting and no tag detected, move a little and try to find tag
//                 runtime.reset();
//                 // Drive Backwards to launch position
//                 while(runtime.seconds() <= endTime){
//                     //move(forward, strafe, rotation)
//                     drive.move(forwardPower,0.0,0.0);
//                     //read april tag on goals
//                     position = eyeball.getCurrentPoseArray();
//                 }//end while
//                 drive.brakes();
//                 runtime.reset();
//                 // Drive Backwards to launch position
//                 while(runtime.seconds() <= endTime){
//                     //move(forward, strafe, rotation)
//                     drive.move(0.0,0.0,rotatePower);
//                     //read april tag on goals
//                     position = eyeball.getCurrentPoseArray();
//                 }//end while
//                 drive.brakes();
//                 
//                 initialPositionDetected = true;
//                 
//             }//end if false
//               */  
//                 
//             
//             
//             
//             
//             
//             
//             
//             
//             // determine which target is closer
//             
//             //check distance to each target
//             distanceFront = Math.sqrt( Math.pow( (position[0]- frontTarget[0]) ,2) + Math.pow( (position[1]-frontTarget[1]) ,2) );
//             distanceBack = Math.sqrt( Math.pow( (position[0]- backTarget[0]) ,2) + Math.pow( (position[1]-backTarget[1]) ,2) );
//             
// 
//             /*
//             //calculate targetVector(x,y,phi)
//             
//             if(distanceFront < distanceBack){
//                 chosenTarget = frontTarget;
//                 // robot - target
//                 dx = position[0]- chosenTarget[0];
//                 dy = position[1]- chosenTarget[1];
//                 
//                 // dx,dy gives arctan(dx/dy) ... may switch to dy,dx arctan(dy/dx) --- CHECK
//                 // Math.atan2 returns radians, 180/pi conversion to degrees.
//                 phi = ( Math.atan2(dx,dy) * (180/Math.PI) );
//                 
//                 magTarget = Math.sqrt( Math.pow(dx,2) + Math.pow(dy,2) );
//                 
//                 //build targetVector
//                 targetVector[0] = dx;
//                 targetVector[1] = dy;
//                 targetVector[2] = magTarget;
//                 targetVector[3] = phi;
//             }//end front<back
//             else{
//                 chosenTarget = backTarget;
//                 // robot - target
//                 dx = position[0]- chosenTarget[0];
//                 dy = position[1]-chosenTarget[1];
//                 
//                 // dx,dy gives arctan(dx/dy) ... may switch to dy,dx arctan(dy/dx) --- CHECK
//                 // Math.atan2 returns radians, 180/pi conversion to degrees.
//                 phi = ( Math.atan2(dx,dy) * (180/Math.PI) );
//                 
//                 magTarget = Math.sqrt( Math.pow(dx,2) + Math.pow(dy,2) );
//                 
//                 //build targetVector
//                 targetVector[0] = dx;
//                 targetVector[1] = dy;
//                 targetVector[2] = magTarget;
//                 targetVector[3] = phi;
//             }//end else
//             */
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
//             //turn until yaw equals phi within headingError
//             
//             // PIDController differenceYawPhi
//             pidValue = pid.calculatePID(position[5],targetVector[3]);
//             // if yaw-phi greater than errorAngle
//             if( (position[5]-targetVector[3]) > errorAngle){
//                 // set motor rotate PID value
//                 //move(forward, strafe, rotation)
//                     
//                     while( (position[5]-targetVector[3]) > errorAngle){
//                         drive.move(0.0,0.0,pidValue/4);
//                          odocomp.update();
//                          currentX = odocomp.getPosX(DistanceUnit.INCH);
//                          currentY = odocomp.getPosY(DistanceUnit.INCH);
//                          currentHeading = odocomp.getHeading(AngleUnit.DEGREES);
//                          position[0]=currentX;
//                          position[1]=currentY;
//                          position[5]=currentHeading;
//                          pidValue = pid.calculatePID(position[5],targetVector[3]);
//                          telemetry.addData("angle", position[5]);
//                          telemetry.addData("target", targetVector[3]);
//                          telemetry.addData("difference", (position[5]-targetVector[3]));
//                          telemetry.addData("pidValue",pidValue);
//                          telemetry.update();
//                         
//                         
//                     }//end while
//             }//end if > errorAngle
//             //reset PID to zero
//             else{
//             // else rotate power zero
//                 drive.move(0.0,0.0,0.0);
//                 drive.brakes();
//             }//end else
//             pid.resetPID();
// 
//                     
//                     
//                     
//                     
//                     
//                     
//                     
//             //if < error angle then drive
//             //drive forward to target
//             if( (position[5]-targetVector[3]) <= errorAngle){        
//                 // PIDController magnitudetargetVector
//                 pidValue = pid.calculatePID(0.0, targetVector[2]);
//                 // check for obstacles ---------- NEED TO ADD then change IFs below to incorporate
//                 //if no obstacles (not implemented yet) and magnitudetargetVector greater than errorDistance
//                 if(targetVector[2] > errorDistance){
//                     //set motor power to PID Value
//                     drive.move(pidValue/100,0.0,0.0);
//                 }//end >errorDistance
//                 else{
//                 // if magnitudetargetVector equal or less than errorDistance
//                             //set motor power to zero, brake
//                     drive.move(0.0,0.0,0.0);
//                     drive.brakes();
//                 }//end else
//                 pid.resetPID();
//                                 
//                 //if obstacles run avoidance pattern  -------- NEED TO ADD
//                         
//                         
//             }//end if =< errorAngle
//                     
//                     
//                         
//             odocomp.update();
//             currentX = odocomp.getPosX(DistanceUnit.INCH);
//             currentY = odocomp.getPosY(DistanceUnit.INCH);
//             currentHeading = odocomp.getHeading(AngleUnit.DEGREES);
//             
//             // if the camera detects april tags on the goals, use them to set current position.
//             if(position[6] == 20 || position[6] == 24){
//                 currentX = position[0];
//                 currentY = position[1];
//                 currentHeading = position[5];
//             }//end if detects april tags                 
//             
//             // if at targetlocation spin until yaw equals preset value for launching
//                 //launch sequence - spin motor, push artifact of correct color, rinse,repeat x3
//                 
//             // if launch sequence was run - strafe preset distance away from launch line and stop
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
//             //  TELEMTRY DISPLAY
//             //telemetry.addData("currentX",currentX);
//             //telemetry.addData("currentY", currentY);
//             //telemetry.addData("currentHeading",currentHeading);
//             //telemetry.update();
//             
//             
//             
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