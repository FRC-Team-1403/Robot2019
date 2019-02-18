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

public class MoveArm extends Command {
  boolean usePID = true;
  public MoveArm() {
    // Use requires() here to declare subsystem dependencies
    requires(Robot.arm);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {

    if(SetControl.mode == 1){
      Robot.arm.moveBy(Robot.m_oi.ojoy.getRawAxis(1));

      //if(Robot.m_oi.ojoy.getRawAxis(RobotMap.ojoyRY) == 0)
        //Robot.w.moveByArm(Robot.arm.voltToRadians(Robot.arm.potentiometerArm.getAverageVoltage()));  
    }
    
    if(SetControl.mode == 2){
      Robot.arm.setpoint = Robot.arm.ballPositions[SetControl.ballLevel].getArmAngle();
      if(Robot.m_oi.ojoy.getRawAxis(5) == 0)
        Robot.w.setpoint = Robot.arm.ballPositions[SetControl.ballLevel].getWristAngle();
    }

    else if(SetControl.mode == 3){
      Robot.arm.setpoint = Robot.arm.hatchPositions[SetControl.hatchLevel].getArmAngle();
      if(Robot.m_oi.ojoy.getRawAxis(5) == 0)
        Robot.w.setpoint = Robot.arm.hatchPositions[SetControl.hatchLevel].getWristAngle();
    }

    Robot.arm.PID();
    Robot.arm.armTest(-Robot.arm.PID);    


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
