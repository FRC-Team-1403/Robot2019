/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class SetControl extends Command {

  static int ballLevel = 1;
  // 1 - floor
  // 2 - human player station
  // 3 - 1st
  // 4 - 2nd
  // 5 - 3rd
  
  static int hatchLevel = 1;
  // 1 - floor
  // 2 - 1st
  // 3 - 2nd
  // 4 - 3rd

  public SetControl() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if (SwitchMode.mode == 1)
      continuousControl();
    if (SwitchMode.mode == 2)
      setBallControl();
    if (SwitchMode.mode == 3)
      setHatchControl();
  }

  public void continuousControl() {
    Robot.w.moveWithArm();
  }

  public void setBallControl() {
    if (Robot.m_oi.ojoy.getRawAxis(5) > 0.5) {
      ballLevel++;
      if (ballLevel > 5) {
        ballLevel = 5;
      }
    }

    if (Robot.m_oi.ojoy.getRawAxis(5) < -0.5) {
      ballLevel--;
      if (ballLevel < 1) {
        ballLevel = 1;
      }
    }

    switch (ballLevel) {
      case 1: Robot.arm.setSetpoint(Robot.arm.floorAngle); break;
      case 2: Robot.arm.setSetpoint(Robot.arm.hpAngle); break;
      case 3: Robot.arm.setSetpoint(Robot.arm.firstAngle); break;
      case 4: Robot.arm.setSetpoint(Robot.arm.secondAngle); break;
      case 5: Robot.arm.setSetpoint(Robot.arm.thirdAngle); break;
    }

    Robot.w.moveWithArm();
  }

  public void setHatchControl() {
    Robot.w.setSetpoint(90.0f);

    if (Robot.m_oi.ojoy.getRawAxis(5) > 0.5) {
      hatchLevel++;
      if (hatchLevel > 4) {
        hatchLevel = 5;
      }
    }

    if (Robot.m_oi.ojoy.getRawAxis(5) < -0.5) {
      hatchLevel--;
      if (hatchLevel < 1) {
        hatchLevel = 1;
      }
    }

    switch (ballLevel) {
      case 1: Robot.arm.setSetpoint(Robot.arm.floorAngle); break;
      case 2: Robot.arm.setSetpoint(Robot.arm.firstAngle); break;
      case 3: Robot.arm.setSetpoint(Robot.arm.secondAngle); break;
      case 4: Robot.arm.setSetpoint(Robot.arm.thirdAngle); break;
    }
        
    Robot.w.moveWithArm();
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
