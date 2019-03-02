/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.Setpoint;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SetControl extends Command {
  public static int ballLevel = 1; // 1 - floor, 2 - 1st, 3 - 2nd, 4 - 3rd
  public static int hatchLevel = 1; // 1 - floor, 2 - 1st, 3 - 2nd, 4 - 3rd
  public static int mode = 0; //0 - continuous, 2 - ball control, 1 - hatch control
  boolean currentlyPressed = false;
  //angles are relative to the flat

  public final Setpoint[] hatchPositions = {new Setpoint(4.353026898, 2.130126735,true), new Setpoint(4.401855018, 0.5932616580000001), new Setpoint(3.455810193, 1.408691262), new Setpoint(2.6879880060000003, 2.1118161900000003, true)};
  public final Setpoint[] ballPositions = {new Setpoint(4.210204647, 2.51464818),new Setpoint(4.401855018, 0.5932616580000001), new Setpoint(3.922118739, 2.504882556), new Setpoint(2.935790715, 3.447265272), new Setpoint(2.302245858, 3.7890621120000003, true)};

  public SetControl() {
    // Use requires() here to declare subsystem dependencies
    requires(Robot.cs);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    updatePotentiometerReadings(Robot.arm.flat, Robot.w.flat);
  }
  
  boolean joystickMoved() {
    double stick = Robot.m_oi.ojoy.getRawAxis(RobotMap.ojoyLY);
    if(currentlyPressed && Math.abs(stick) > .5) {
      return false;
    } else if(currentlyPressed && Math.abs(stick) < .5) {
      currentlyPressed = false;
      return false;
    } else if(!currentlyPressed && Math.abs(stick) > .5) {
      currentlyPressed = true;
      return true;
    }
    return false;
  }

  public void updatePotentiometerReadings(double aPotReading, double wPotReading){
    Robot.arm.flat = aPotReading;
    Robot.w.flat = wPotReading;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if(Robot.m_oi.ojoy.getRawButtonPressed(RobotMap.ojoyStart)) {
      mode++;
    }

    mode %= 3;
    
    if(Robot.m_oi.tjoy.getRawButtonPressed(RobotMap.ojoyBack)) {
      updatePotentiometerReadings(Robot.arm.potentiometerArm.getAverageVoltage(), Robot.w.potentiometerWrist.getAverageVoltage());
    } 

      if(joystickMoved()) {
     
        if(Robot.m_oi.ojoy.getRawAxis(RobotMap.ojoyLY) < -.5) {
          if(mode == 1) {
           ballLevel++;        
           if(ballLevel == ballPositions.length) {
             ballLevel = ballPositions.length-1;
           }
           ballPositions[ballLevel].run();
         }
         if(mode == 2) {
           hatchLevel++;
           if(hatchLevel == hatchPositions.length) {
             hatchLevel = hatchPositions.length-1;
            }
           hatchPositions[hatchLevel].run(); 
         }
       } else {
         if(mode == 1) {
           ballLevel--;
            if(ballLevel == -1){
             ballLevel = 0;
           }
           ballPositions[ballLevel].run();        
         }
         if(mode == 2) {
           hatchLevel--;
           if(hatchLevel == -1) {
             hatchLevel = 0;
           }
           hatchPositions[hatchLevel].run(); 
        }
      }
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
