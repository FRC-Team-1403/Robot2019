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
     if(Robot.m_oi.ojoy.getRawAxis(RobotMap.ojoyRY) != 0)
      Robot.w.wristTest(-Robot.m_oi.ojoy.getRawAxis(RobotMap.ojoyRY));
     else{
      Robot.w.PID();
      //Robot.w.wristTest(Robot.w.PID);
     }
    /*if(Robot.m_oi.ojoy.getRawButton(RobotMap.ojoyBack))
      Robot.w.armConversion -= .001;
     else if(Robot.m_oi.ojoy.getRawButton(RobotMap.ojoyStart))
      Robot.w.armConversion += .001;
     if(Robot.m_oi.ojoy.getRawButton(RobotMap.ojoyA))
      Robot.w.armConversion += .01;
     else if(Robot.m_oi.ojoy.getRawButton(RobotMap.ojoyX))
      Robot.w.armConversion -= .01;
     if(Robot.m_oi.ojoy.getRawButton(RobotMap.ojoyY))
      Robot.w.armConversion += .1;
     else if(Robot.m_oi.ojoy.getRawButton(RobotMap.ojoyB))
      Robot.w.armConversion -= .1;*/
  
}
// Robot.w.PID();
//     Robot.w.wristTest(-Robot.m_oi.ojoy.getRawAxis(RobotMap.ojoyRY));
//   }

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
