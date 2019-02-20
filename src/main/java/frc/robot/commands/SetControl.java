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
  public static final double[][] ballAngles = {{.75087152622, -.65794353926}, {0.39339194857, -.704520682933}, {2.21688712317, -.24690024639}, {1.43904882392, .6939580557}, {0.76833795509, 1.29829649479}};
  public static final double[][] hatchAngles = {{0.38407651983, -.65794353926}, {0.39339194857, -.704520682933}, {-0.50554692422,.19791147563}, {-0.83158692989, .93033705981}};
  
  public static Setpoint[] hatchPositions = new Setpoint[hatchAngles.length];
  public static Setpoint[] ballPositions = new Setpoint[ballAngles.length];



  public SetControl() {
    // Use requires() here to declare subsystem dependencies
    for(int i=0; i<hatchAngles.length; i++) {
      hatchPositions[i] = new Setpoint(hatchAngles[i][0], hatchAngles[i][1]);
    } 
    for(int i = 0; i < ballAngles.length; i++){
      ballPositions[i] = new Setpoint(ballAngles[i][0], ballAngles[i][1]);
    }

    hatchPositions[0].armOut = true;
    hatchPositions[3].armOut = true;
    ballPositions[4].armOut = true;

    requires(Robot.cs);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    updatePotentiometerReadings(Robot.arm.flat);
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

  public void updatePotentiometerReadings(double potentiometerReading){
    Robot.arm.flat = potentiometerReading;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if(Robot.m_oi.ojoy.getRawButtonPressed(RobotMap.ojoyStart)) {
      mode++;
    }

    mode %= 3;
    
    if(Robot.m_oi.ojoy.getRawButtonPressed(RobotMap.ojoyBack)) {
      updatePotentiometerReadings(Robot.arm.potentiometerArm.getAverageVoltage());
    } //DONT FORGET TO SET TO A SEPARATE JOYSTICK

      if(joystickMoved()) {
     
        if(Robot.m_oi.ojoy.getRawAxis(RobotMap.ojoyLY) < -.5) {
          if(mode == 2) {
           ballLevel++;        
           if(ballLevel == ballPositions.length) {
             ballLevel = ballPositions.length-1;
           }
           ballPositions[ballLevel].run();
         }
         if(mode == 1) {
           hatchLevel++;
           if(hatchLevel == hatchPositions.length) {
             hatchLevel = hatchPositions.length-1;
            }
           hatchPositions[hatchLevel].run(); 
         }
       } else {
         if(mode == 2) {
           ballLevel--;
            if(ballLevel == -1){
             ballLevel = 0;
           }
           ballPositions[ballLevel].run();        
         }
         if(mode == 1) {
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
