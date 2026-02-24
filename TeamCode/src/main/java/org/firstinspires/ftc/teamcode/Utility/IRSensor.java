// package org.firstinspires.ftc.teamcode.Utility;
// 
// import com.qualcomm.robotcore.hardware.HardwareMap;
// import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
// import com.qualcomm.robotcore.hardware.DistanceSensor;
// 
// 
// public class IRSensor {
//  
//  
//     
//     //safe distances for each sensor 
//     // measured in CM
//     double safeBS1 = 5.0;
//     double safeBS2 = 5.0;
//     double safeLS1 = 5.0;
//     double safeLS2 = 5.0;
//     double safeRS1 = 5.0;
//     double safeRS2 = 5.0;
//     double safeFS1 = 5.0;
//     double safeFS2 = 5.0;
//     
//     
//     //puts the safe distances together in an array
//     //ORDER OF BS, FS, RS, LS MUST MATCH detectedDistances array
//     double[] safeArray = {safeBS1,safeBS2, safeFS1,safeFS2, safeRS1,safeRS2, safeLS};
//     
//     
//     //empty array to hold true/false in detectObstacles method
//     boolean[] obstacleDetected = {false,false,false,false,false,false,false,false};
//  
//  
//     //BSensor is Back Sensor
//     public DistanceSensor BSensor1;
//     public DistanceSensor BSensor2;
//     //Lsensor is Left Sensor
//     public DistanceSensor LSensor1;
//      public DistanceSensor FSensor2;
// // 
//     //Rsensor is Right Sensor
//     public DistanceSensor RSensor1;
//      public DistanceSensor RSensor2;
//     //FSensor is Front Sensor
//     public DistanceSensor FSensor1;
//       public DistanceSensor FSensor2;
// 
// 
// 
//     //BSValue is Back Sensor Value
//     double BSValue1;
//      double BSValue2;
//     //LSValue is Left Sensor Value
//     double LSValue1;
//     double LSValue2;
//     //RSValue is Right Sensor Value
//     double RSValue1;
//      double RSValue2;
//     //FSValue is Front Sensor Value
//     double FSValue1;
//     double FSValue2;
//   
//   
//     double[] detectedDistances = {0.0,0.0,0.0,0.0};
//     double[] currentDistances = {0.0,0.0,0.0,0.0};
//   
// 
// 
// 
//   public void initSensor(HardwareMap hwmap){
// 
//     BSensor1 = hwmap.get(DistanceSensor.class, "BDistance1");
//     BSensor2 = hwmap.get(DistanceSensor.class, "BDistance2");
//     LSensor1 = hwmap.get(DistanceSensor.class, "LDistance");
//     LSensor2 = hwmap.get(DistanceSensor.class, "LDistance2");
//     RSensor1 = hwmap.get(DistanceSensor.class, "RDistance1");
//     RSensor2 = hwmap.get(DistanceSensor.class, "RDistance2");
//     FSensor1 = hwmap.get(DistanceSensor.class, "FDistance1");
//     FSensor2 = hwmap.get(DistanceSensor.class, "FDistance2");
//     }//end init method
// 
// 
// 
// 
// 
// 
//   public double[] getValue(){
//     
//     BSValue1 = BSensor1.getDistance(DistanceUnit.CM);
//     BSValue2 = BSensor2.getDistance(DistanceUnit.CM);
//     RSValue1 = RSensor1.getDistance(DistanceUnit.CM);
//     RSValue2 = RSensor2.getDistance(DistanceUnit.CM);
//     LSValue1 = LSensor1.getDistance(DistanceUnit.CM);
//     LSValue2 = LSensor2.getDistance(DistanceUnit.CM);
//     FSValue1 = FSensor1.getDistance(DistanceUnit.CM);
//     FSValue2 = FSensor2.getDistance(DistanceUnit.CM);
//     
//     detectedDistances[0] = BSValue;
//     detectedDistances[1] = FSValue;
//     detectedDistances[2] = RSValue;
//     detectedDistances[3] = LSValue;
//     
//     return detectedDistances;
//     } // end getvalue method
//   
//   
//   
//   
//   
//   
//   public boolean[] detectObstacles(){
//       
//     currentDistances = getValue();
//     
//     for(int i = 0; i < 4; i ++){
//         if(currentDistances[i] <= safeArray[i]){
//             obstacleDetected[i] = false;
//         }//end if =<
//         
//         if(currentDistances[i] > safeArray[i]){
//             obstacleDetected[i] = true;
//         }// end if >
//     }//end for safearray
//     
//     return obstacleDetected;
//     }//end obstacleDetected method
//   
//   
// }// end class
// 