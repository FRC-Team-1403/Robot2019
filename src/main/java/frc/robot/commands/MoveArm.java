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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class MoveArm extends Command {
  public MoveArm() {
    requires(Robot.arm);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if(Robot.m_oi.ojoy.getRawButtonPressed(RobotMap.ojoyRB)){
      Robot.arm.setpoint = Robot.arm.angle;
    }
    
    Robot.arm.moveBy(Robot.m_oi.ojoy.getRawAxis(1));
    /*if(Robot.m_oi.tjoy.getRawButton(RobotMap.ojoyA)){
      Robot.arm.P += .01;
    }
    else if(Robot.m_oi.tjoy.getRawButton(RobotMap.ojoyB)){
      Robot.arm.P -= .01;
    }
    if(Robot.m_oi.tjoy.getRawButton(RobotMap.ojoyLB)){
      Robot.arm.I += .001;
    }

    else if(Robot.m_oi.tjoy.getRawButton(RobotMap.ojoyRB)){
      Robot.arm.I -= .001;
    }
    if(Robot.m_oi.tjoy.getRawButton(RobotMap.ojoyX)){
      Robot.arm.D += .002;
    }
    else if(Robot.m_oi.tjoy.getRawButton(RobotMap.ojoyY)){
      Robot.arm.D -= .002;
    }
    if(Robot.m_oi.tjoy.getRawButton(RobotMap.ojoyBack)){
      Robot.arm.armConstant -= .001;
    }
    else if(Robot.m_oi.tjoy.getRawButton(RobotMap.ojoyStart)){
      Robot.arm.armConstant += .001;
    }
    SmartDashboard.putNumber("Arm Adjustment", Robot.arm.armConstant);
    SmartDashboard.putNumber("Arm P", Robot.arm.P);
    SmartDashboard.putNumber("Arm I", Robot.arm.I);
    SmartDashboard.putNumber("Arm D", Robot.arm.D);
*/
    Robot.arm.PID();
    Robot.arm.moveArm(-Robot.arm.PID);    
    //Robot.arm.moveArm(Robot.m_oi.ojoy.getRawAxis(1));    


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
