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
import frc.robot.commands.IntakeC;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Robot;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.Servo;

/**
 * Add your docs here.
 */
public class Intake extends Subsystem {
  public VictorSPX intakeMotor;
  public DoubleSolenoid hatchPush;
  public PWM hook;
  public double value;

  public Intake(){
    intakeMotor = new VictorSPX(RobotMap.intakeMotor);
    hatchPush = new DoubleSolenoid(RobotMap.hatchPush1, RobotMap.hatchPush2);
    hook= new PWM(RobotMap.hookServo);
  }
public void intake(){
  if(Robot.m_oi.ojoy.getRawAxis(3)>0)
  {
    intakeMotor.set(ControlMode.PercentOutput, Robot.m_oi.ojoy.getRawAxis(3));
  }
  else if(-Robot.m_oi.ojoy.getRawAxis(2)<0)
  {
    intakeMotor.set(ControlMode.PercentOutput, -Robot.m_oi.ojoy.getRawAxis(2));
  }
  else
  {
    intakeMotor.set(ControlMode.PercentOutput, 0);
  }
}
public void push()
{
  if(Robot.m_oi.ojoy.getRawButton(RobotMap.ojoyRB))
  {
    hatchPush.set(DoubleSolenoid.Value.kForward);
  }
  else if(Robot.m_oi.ojoy.getRawButton(RobotMap.ojoyLB))
  {
    hatchPush.set(DoubleSolenoid.Value.kReverse); 
  }
  else
  {
    hatchPush.set(DoubleSolenoid.Value.kOff);
  }
}
public void hookHatchPanel(){
  if(Robot.m_oi.ojoy.getRawButton(RobotMap.ojoyX)){
    hook.setPosition(1);
  }
  else if(Robot.m_oi.ojoy.getRawButton(RobotMap.ojoyY)){
    hook.setPosition(0);
  }
}

public static void setSpeed(VictorSPX victor, double speed){
  victor.set(ControlMode.PercentOutput, speed);
}

public double convertBoolToDouble()
{
  if(Robot.m_oi.ojoy.getRawButton(RobotMap.ojoyRB))
  {
    return 1.0;
  }
  else if (Robot.m_oi.ojoy.getRawButton(RobotMap.ojoyLB)){
    return -1.0;
  }
  if(Robot.m_oi.ojoy.getRawButton(RobotMap.ojoyX)){
    return 2.0;
  }
  else if(Robot.m_oi.ojoy.getRawButton(RobotMap.ojoyY)){
    return 3.0;
  }
  else {
    return 0.0;
  }

  
}
public static void setPosition(DoubleSolenoid doubleSolenoid, double value)
{
  if (Robot.in.convertBoolToDouble()==1){
    doubleSolenoid.set(DoubleSolenoid.Value.kForward);

  }
  if (Robot.in.convertBoolToDouble()== -1.0){
    doubleSolenoid.set(DoubleSolenoid.Value.kReverse);
  }
  if(Robot.in.convertBoolToDouble() == 0.0){
    doubleSolenoid.set(DoubleSolenoid.Value.kOff);
  } 
}
public static void setServo(PWM servo, double position){
  if(Robot.in.convertBoolToDouble()==2.0){
    servo.setPosition(1);
  }
  if(Robot.in.convertBoolToDouble()==3.0){
    servo.setPosition(0);
  }
}
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    setDefaultCommand(new IntakeC());
  }
}


