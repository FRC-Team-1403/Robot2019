/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.commands.LiftRobot;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import frc.robot.RobotMap;
import frc.robot.Robot;

/**
 * Add your docs here.
 */
public class MoveLiftingPiston extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  public DoubleSolenoid lifter;

  public MoveLiftingPiston(){
    lifter = new DoubleSolenoid(RobotMap.lifter1, RobotMap.lifter2);
  }

  public void moveLiftingPiston(){
    if(Robot.m_oi.tjoy.getRawButtonPressed(RobotMap.ojoyA)){
      lifter.set(DoubleSolenoid.Value.kForward);
    }
    else if (Robot.m_oi.tjoy.getRawButtonPressed(RobotMap.ojoyB)){
      lifter.set(DoubleSolenoid.Value.kReverse);
    }
    else{
      lifter.set(DoubleSolenoid.Value.kOff);
    }
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    setDefaultCommand(new LiftRobot());
  }
}
