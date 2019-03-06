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

  public static DoubleSolenoid lifter;

  public MoveLiftingPiston(){
    lifter = new DoubleSolenoid(RobotMap.lifter1, RobotMap.lifter2);
  }

  public static void moveLiftingPiston(){
    if(Robot.m_oi.tjoy.getRawButtonPressed(RobotMap.ojoyA)){
      lifter.set(DoubleSolenoid.Value.kForward);
    }
    if (Robot.m_oi.tjoy.getRawButtonPressed(RobotMap.ojoyB)){
      lifter.set(DoubleSolenoid.Value.kReverse);
    }
  }
  public static double convertBoolToDouble(){
    if(Robot.m_oi.tjoy.getRawButton(RobotMap.ojoyBack)){
      return 1.0;
    }
    else if(Robot.m_oi.tjoy.getRawButton(RobotMap.ojoyStart)){
      return 2.0;
    }
    else{
      return 0.0;
    }
  }
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    setDefaultCommand(new LiftRobot());
  }
}
