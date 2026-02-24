package org.firstinspires.ftc.teamcode.Utility;

import org.firstinspires.ftc.robotcore.external.Telemetry;


public class ArenaMap{

    //arena is 365.75cm squared (approx)
    int arenaSize = 365;

    int goalSize = 69;
    


    //creates an empty array of type boolean
    boolean[][] arenaGrid;

    //fills arenaGrid array with cells each unit in arenaSize
    public void arenaGridBuilder(){

        arenaGrid = new boolean[arenaSize][arenaSize];
        
                
        //fill arenagrid with true booleans

        for( int row = 0; row<arenaSize; row++){
            for (int column = 0; column<arenaSize; column++){
                arenaGrid[row][column] = true;
            }//end for loop
        }//end for loop

        //add in obstacles defined in arenaObstacles method
        arenaObstacles();

    }//end arenaDecodeLayout

    
    
    
    

    //method to return arenaGrid array to other classes (i.e. opmode)

    public boolean[][] arenaGridGetter(){
        return arenaGrid;
    }//end arenaGridGetter









    //print the arenaGrid to the telemtry window
    
    public void printGrid(Telemetry telemetry, int printStart, int printEnd){
        
            printStart = printStart;
    
            //no more than +25 from print Start
            printEnd = printEnd;

        telemetry.addLine("--arenaGrid--");
        telemetry.addData("from",printStart);
        telemetry.addData("to",printEnd);

        // Display each row as a string of T (true) and F (false)

        for (int row = printStart; row < printEnd; row++) {

            StringBuilder rowPrintValues = new StringBuilder();

            for (int column = printStart; column < printEnd; column++) {
                rowPrintValues.append(arenaGrid[row][column] ? "T " : "F ");
            } //end for loop

            telemetry.addLine(rowPrintValues.toString().trim());
        } //end for loop

        telemetry.update();
    
    }//end printMap









    // blocks of non-moving obstacles like goals
    public void arenaObstacles(){
        
        //block off goals
        
        //blue goal (0 -> 69cm (0 -> 68)row, 0 -> 69cm (0 -> 68)column, triangle)

        for( int row = 0; row < goalSize; row++){
            for (int column = 0; column < goalSize; column++){
                if( row+column < goalSize ){
                    arenaGrid[row][column] = false;
                }//end if statement
            }//end for loop
        }//end for loop


        //redgoal (0 -> 69cm (0 -> 68)row, 296 -> 365cm (295 -> 364)column, triangle)

        for( int row = 0; row < goalSize; row++){
            for (int column = 295; column < arenaSize; column++){
                if( (column-row >= 295) && (row+column < arenaSize) ){
                    arenaGrid[row][column] = false;
                }//end if statement
            }//end for loop
        }//end for loop



    

        //block off ramp/classifier (not gate handle)

        //red ramp/tunnel (0 -> 16cm (0 -> 15)column, 0 -> 306cm (0 -> 305)row, rectangle)
        
        for (int row = 0; row < 306; row++){
            for (int column = 0; column < 16; column++){
                arenaGrid[row][column] = false;
            }//end for loop
        }//end for loop

        


        //blue ramp/tunnel ( 0 -> 306cm (0 -> 305)row, 349 -> 365cm (348 -> 364)column, rectangle)
        
        for (int row = 0; row < 306; row++){
            for (int column = 348; column < 365; column++){
                arenaGrid[row][column] = false;
            }//end for loop
        }//end for loop 
        
        
    
        

    } //end arenaObstacles






    //checks to see if row/column values are within arena size

    public boolean inBounds(int gridRow, int gridColumn){
        //provides true/false if row/column values are within arena size
        return gridRow >= 0 && gridRow < arenaSize 
            && gridColumn >= 0 && gridColumn < arenaSize;

    } //end inBounds



} //end class

