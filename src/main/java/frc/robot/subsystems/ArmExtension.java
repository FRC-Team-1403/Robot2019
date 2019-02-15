/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Robot;
import frc.robot.commands.Extend;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class ArmExtension extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  public DoubleSolenoid armExtender;

  public ArmExtension() {
    armExtender = new DoubleSolenoid(RobotMap.extensionSolenoid1, RobotMap.extensionSolenoid2);
  }

  public void push() {
    if(Robot.m_oi.ojoy.getRawButton(RobotMap.ojoyA)) 
    {
      armExtender.set(DoubleSolenoid.Value.kForward);
    }
    else if(Robot.m_oi.ojoy.getRawButton(RobotMap.ojoyB)) 
    {
      armExtender.set(DoubleSolenoid.Value.kReverse);
    }
    else 
    {
      armExtender.set(DoubleSolenoid.Value.kOff);
    }
  }
  public static void setPosition(DoubleSolenoid doubleSolenoid, double value){ 
  if (value==1.0){
    doubleSolenoid.set(DoubleSolenoid.Value.kForward);
  }
  if (value == -1.0){
    doubleSolenoid.set(DoubleSolenoid.Value.kReverse);
  }
  if(value == 0.0){
    doubleSolenoid.set(DoubleSolenoid.Value.kOff);
  }
}

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    setDefaultCommand(new Extend());
  }
}
