/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import javax.lang.model.util.ElementScanner6;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import frc.robot.RobotMap;
import frc.robot.commands.IntakeCommand;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Robot;
import edu.wpi.first.wpilibj.DoubleSolenoid;

/**
 * Add your docs here.
 */
public class Intake extends Subsystem {
  public VictorSPX intakeMotor;
  public DoubleSolenoid hatchPush;

  public Intake(){
    intakeMotor = new VictorSPX(RobotMap.intakeMotor);
    hatchPush = new DoubleSolenoid(RobotMap.hatchPush1, RobotMap.hatchPush2);
  }
public void intake(){
  if(Robot.m_oi.ojoy.getRawAxis(3)>0)
  {
    intakeMotor.set(ControlMode.PercentOutput, .5*Robot.m_oi.ojoy.getRawAxis(3));
  }
  else if(-Robot.m_oi.ojoy.getRawAxis(2)<0)
  {
    intakeMotor.set(ControlMode.PercentOutput, -.5*Robot.m_oi.ojoy.getRawAxis(2));
  }
  else
  {
    intakeMotor.set(ControlMode.PercentOutput, 0);
  }
}
public void push(){
    hatchPush.set(DoubleSolenoid.Value.kReverse);

  }
public void release(){
    hatchPush.set(DoubleSolenoid.Value.kForward);
  }
  

  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
   setDefaultCommand(new IntakeCommand());
  }
}
