package org.firstinspires.ftc.teamcode.Utility;

public class Calculations {


    public double[] calcDiffVec(double[] currentPosition, double[] targetPosition){

        double[] position = currentPosition;
        double[] targetPos = targetPosition;




        //calculate targetVector(x,y,phi)
        double dx = position[0] - targetPos[0];
        double dy = position[1]- targetPos[1];
        //  dy,dx gives arctan(dy/dx)
        // Math.atan2 returns radians, 180/pi conversion to degrees.
        double phi = ( Math.atan2(dy,dx) * (180/Math.PI) );
        double magTarget = Math.sqrt( Math.pow(dx,2) + Math.pow(dy,2) );

        double angleTurn = calcTurnAngle(position, targetPos, phi);

        double[] targetVec = {dx,dy,phi,magTarget, angleTurn};

        return targetVec;
    }


    public double calcTurnAngle(double[] currentPosition, double[] targetPosition, double Phi){

        double[] targetPos = targetPosition;
        double[] position = currentPosition;
        double phi = Phi;
        double turnAngle;

        //target in Q1
        if( (targetPos[0]>0) && (targetPos[1]>0) ){
            //phi (+), arenaAngleCoord (-)
            turnAngle = 270 + phi;
            //if required to turn more than 180, turn opposite direction
            //towards desired turnAngle
            if((turnAngle - position[5])>180){
                turnAngle = turnAngle - 360;
            }
            return turnAngle;
        }


        //target in Q2
        if( (targetPos[0]<0) && (targetPos[1]>0) ){
            //phi (-), arenaAngleCoord (+)
            turnAngle = 90 + phi;
            //if required to turn more than 180, turn opposite direction
            //towards desired turnAngle
            if((turnAngle - position[5])>180){
                turnAngle = turnAngle - 360;
            }
            return turnAngle;
        }


        //target in Q3
        if( (targetPos[0]<0) && (targetPos[1]<0) ){
            //phi (+), arenaAngleCoord (+)
            turnAngle = 90 + phi;
            //if required to turn more than 180, turn opposite direction
            //towards desired turnAngle
            if((turnAngle - position[5])>180){
                turnAngle = turnAngle - 360;
            }
            return turnAngle;
        }


        //target in Q4
        if( (targetPos[0]>0) && (targetPos[1]<0) ){
            //phi (-), arenaAngleCoord (-)
            turnAngle = 270 + phi;
            //if required to turn more than 180, turn opposite direction
            //towards desired turnAngle
            if((turnAngle - position[5])>180){
                turnAngle = turnAngle - 360;
            }
            return turnAngle;
        }


    }


}
