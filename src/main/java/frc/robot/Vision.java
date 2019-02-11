/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot;


import frc.robot.Robot;



/**
 * Add your docs here.
 */
public class Vision {

    private static boolean inMiddle;
    public static double angleLenience = 3;
    public static final double propPID = 0.01;
    public static final double derivPID = 0.1;
    private static double prevAngle;
    
    public static void alignLeft(){
        if (inMiddle) {
            if(Robot.ta.getDouble(0.0) < 15){
                System.out.println("tilt left");
                Robot.drivetrain.moveForward();
            }
            else{
                //Robot.drivetrain.stop();
                System.out.println("stop");
            }
        } else {
            Robot.drivetrain.turnLeft();
            System.out.println("Left");
        }   
    }

    public static void alignRight(){
        if (inMiddle) {
            if(Robot.ta.getDouble(0.0) < 15){
           
             System.out.println("tilt right");
               
             Robot.drivetrain.moveForward();
            }
            else{
              
                System.out.println("stop");
            }
           
        } else {
            Robot.drivetrain.turnRight();
            System.out.println("Right");
        }   
    }

    public static void align(double angle) { 
        
        if (angle <= 0) {
            alignLeft();
        }
        else{
            alignRight();
        }
    }
    
    public static void isInMiddle(double angle) {
       
        inMiddle = (Math.abs(angle) <= angleLenience) || inMiddle;
    }

    public static void tiltLeft(double angle) {
        double adjustment = angle * propPID - (angle - prevAngle) * derivPID;
        Robot.drivetrain.setRaw(0.75 - adjustment, 0.75);
        prevAngle = angle;
    }

    public static void tiltRight(double angle) {
        double adjustment = angle * propPID - (angle - prevAngle) * derivPID;
        Robot.drivetrain.setRaw(0.75, 0.75 - adjustment);
        prevAngle = angle;
    }

    public static void reset(){
        inMiddle = false;
        Robot.drivetrain.stop();
    }
    
}

   
