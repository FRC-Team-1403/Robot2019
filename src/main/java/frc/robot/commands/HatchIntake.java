/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotMap;

public class HatchIntake extends Command {
  public HatchIntake() {
    requires(Robot.hatch);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if(Robot.m_oi.djoy.getRawButtonPressed(RobotMap.ojoyA)){
      //if(Robot.hatch.hookServo.getPosition() != 1){
        Robot.hatch.push();
      //}
    }
    else if(Robot.m_oi.djoy.getRawButtonReleased(RobotMap.ojoyA)){
      Robot.hatch.release();
    }
    else{
      Robot.hatch.off();
    }
    if(Robot.m_oi.djoy.getRawButtonPressed(RobotMap.ojoyRB)){
      Robot.hatch.hookServo.setPosition(1.0);
    }
    else if(Robot.m_oi.djoy.getRawButtonPressed(RobotMap.ojoyLB))
    {
      Robot.hatch.hookServo.setPosition(0);
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
