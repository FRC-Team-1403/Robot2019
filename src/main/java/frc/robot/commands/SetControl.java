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
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
public class SetControl extends Command {

  public static int ballLevel = 1;
  // 1 - floor
  // 2 - 1st
  // 3 - 2nd
  // 4 - 3rd
  
  public static int hatchLevel = 1;
  // 1 - floor
  // 2 - 1st
  // 3 - 2nd
  // 4 - 3rd

  public static int mode = 1;
  //1 - continuous
  //2 - ball control
  //3 - hatch control  

  boolean currentlyPressed = false;

  public SetControl() {
    // Use requires() here to declare subsystem dependencies

    requires(Robot.cs);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }
  
  boolean joystickMoved() {
    double stick = Robot.m_oi.ojoy.getRawAxis(RobotMap.ojoyLY);
    if(currentlyPressed && Math.abs(stick) > .5){
      return false;
    }

    else if(currentlyPressed && Math.abs(stick) < .5){
      currentlyPressed = false;
      return false;

    }

    else if(!currentlyPressed && Math.abs(stick) > .5){
      currentlyPressed = true;
      return true;
    }

    return false;
    

  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    SmartDashboard.putNumber("Raw axis: ", Robot.m_oi.ojoy.getRawAxis(RobotMap.ojoyLY));
    SmartDashboard.putBoolean("currently down", currentlyPressed);
    if(Robot.m_oi.ojoy.getRawButtonPressed(RobotMap.ojoyStart))
      mode++;
    if (mode > 3)
      mode = 1;
    
    if(joystickMoved()){
      if(Robot.m_oi.ojoy.getRawAxis(RobotMap.ojoyLY) < -.5){
      
      if(mode == 2){
        ballLevel++;  
        
        if(ballLevel == 5)
          ballLevel = 4;
          Robot.arm.setpoint = Robot.arm.ballPositions[SetControl.ballLevel].getArmAngle();
          Robot.w.setpoint = Robot.arm.ballPositions[SetControl.ballLevel].getWristAngle();
            
          if(ballLevel == 4)
          Robot.in.hatchPush.set(DoubleSolenoid.Value.kForward);
        else
        Robot.in.hatchPush.set(DoubleSolenoid.Value.kOff);
      }
      if(mode == 3){
        hatchLevel++;
        if(hatchLevel == 5)
          hatchLevel = 4;
        Robot.arm.setpoint = Robot.arm.hatchPositions[SetControl.hatchLevel].getArmAngle();
        Robot.w.setpoint = Robot.arm.hatchPositions[SetControl.hatchLevel].getWristAngle();
         
        if(hatchLevel == 4)
        Robot.in.hatchPush.set(DoubleSolenoid.Value.kForward);
        else
        Robot.in.hatchPush.set(DoubleSolenoid.Value.kOff);
      }
    }
    
    else{
      if(mode == 2){
        if(ballLevel == 4){
          Robot.in.hatchPush.set(DoubleSolenoid.Value.kReverse);
        }
        ballLevel--;
        if(ballLevel == 0)
        ballLevel = 1;
        Robot.arm.setpoint = Robot.arm.ballPositions[SetControl.ballLevel].getArmAngle();
        Robot.w.setpoint = Robot.arm.ballPositions[SetControl.ballLevel].getWristAngle();
           
    }
      if(mode == 3){
        if(hatchLevel == 4){
        Robot.in.hatchPush.set(DoubleSolenoid.Value.kReverse);
        }
        hatchLevel--;
        if(hatchLevel == 0)
          hatchLevel = 1;
        Robot.arm.setpoint = Robot.arm.hatchPositions[SetControl.hatchLevel].getArmAngle();
        Robot.w.setpoint = Robot.arm.hatchPositions[SetControl.hatchLevel].getWristAngle();
           
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
