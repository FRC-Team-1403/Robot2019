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
  public SetControl() {
    // Use requires() here to declare subsystem dependencies

    requires(Robot.cs);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if(Robot.m_oi.ojoy.getRawButtonPressed(RobotMap.ojoyStart))
      mode++;
    if (mode > 3)
      mode = 1;
    
    if(Robot.m_oi.djoy.getRawButtonPressed(RobotMap.ojoyStart)){
      if(mode == 2){
        ballLevel++;
        if(ballLevel == 5)
          ballLevel = 4;
      }
      if(mode == 3){
        hatchLevel++;
        if(hatchLevel == 5)
          hatchLevel = 4;
      }
    }
    if(Robot.m_oi.djoy.getRawButtonPressed(RobotMap.ojoyBack)){
      if(mode == 2){
        ballLevel--;
      if(ballLevel == 0)
        ballLevel = 1;
    }
      if(mode == 3){
        hatchLevel--;
        if(hatchLevel == 0)
          hatchLevel = 1;
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
