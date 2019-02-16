/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class MoveWrist extends Command {
  public MoveWrist() {
    // Use requires() here to declare subsystem dependencies
    requires(Robot.w);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    /*Robot.w.usePIDOuputVelocity(Robot.m_oi.ojoy.getRawAxis(1));
    if(Math.abs(Robot.m_oi.ojoy.getRawAxis(5)) > 0)
    {
      Robot.w.setSetpointRelative(-Robot.arm.potentiometerArm.getAverageVoltage());
    }*/
    //Robot.w.wristTest();
    
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
