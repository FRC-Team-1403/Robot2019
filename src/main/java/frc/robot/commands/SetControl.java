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
  public static int ballLevel = 0; // 0 - pick up, 1 - HPS, 2 - first level, 3 - second level, 4 - third level
  public static int hatchLevel = 0; // 0 - pick up, 1 - first level, 2 - second level, 3 - third level
  public static int mode = 0; //0 - hatch control, 1 - ball control
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
    //read armPot and wristPot from file or set to default values
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
    Robot.rioIO.writeToRIO(aPotReading, wPotReading);
    Robot.rioIO.readFromRIO();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if(Robot.m_oi.ojoy.getRawButtonPressed(RobotMap.ojoyStart)) {
      mode++;
      mode%=2;
    }

    
    if(Robot.m_oi.tjoy.getRawButtonPressed(RobotMap.ojoyStart)) {
      updatePotentiometerReadings(Robot.arm.potentiometerArm.getAverageVoltage(), Robot.w.potentiometerWrist.getAverageVoltage());
    } 

      if(joystickMoved() && !Robot.m_oi.ojoy.getRawButton(RobotMap.ojoyX)) {
     
        if(Robot.m_oi.ojoy.getRawAxis(RobotMap.ojoyLY) < -.5) {
          if(mode == 1) {
           ballLevel++;        
           if(ballLevel == ballPositions.length) {
             ballLevel = ballPositions.length-1;
           }
           ballPositions[ballLevel].run();
         }
         if(mode == 0) {
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
         if(mode == 0) {
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
