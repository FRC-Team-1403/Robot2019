/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotMap;

public class MoveWrist extends Command {
  public MoveWrist() {
    // Use requires() here to declare subsystem dependencies
    requires(Robot.w);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.w.prevArmAngle = Robot.arm.voltToRadians(Robot.arm.potentiometerArm.getAverageVoltage());
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
     if(Math.abs(Robot.m_oi.ojoy.getRawAxis(RobotMap.ojoyRY)) > .1){
      Robot.w.moveBy(-Robot.m_oi.ojoy.getRawAxis(RobotMap.ojoyRY));
    }
     else{
      if(SetControl.mode == 1)
        Robot.w.moveByArm(Robot.arm.voltToRadians(Robot.arm.potentiometerArm.getAverageVoltage())); 
      
     }
     Robot.w.PID();
     Robot.w.wristTest(Robot.w.PID);
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
