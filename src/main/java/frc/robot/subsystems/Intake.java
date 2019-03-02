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
  public DoubleSolenoid hatchPushSolenoid;
  public Servo hookServo;
  public double value;  
  public double position;  

  public Intake(){
    intakeMotor = new VictorSPX(RobotMap.intakeMotor);
    hatchPushSolenoid = new DoubleSolenoid(RobotMap.hatchPush1, RobotMap.hatchPush2);
    hookServo = new Servo(RobotMap.hookServo);
  }

public void intake(){
  if(Robot.m_oi.ojoy.getRawAxis(2)>0)
  {
    intakeMotor.set(ControlMode.PercentOutput, Robot.m_oi.ojoy.getRawAxis(2));
  }
  else if(-Robot.m_oi.ojoy.getRawAxis(3)<0)
  {
    intakeMotor.set(ControlMode.PercentOutput, -Robot.m_oi.ojoy.getRawAxis(3));
  }
  else
  {
    intakeMotor.set(ControlMode.PercentOutput, 0);
  }
}

public void push(){
  hatchPushSolenoid.set(DoubleSolenoid.Value.kReverse);
}

public void release(){
  hatchPushSolenoid.set(DoubleSolenoid.Value.kForward);
}

public void hookHatchPanel(){
  hookServo.setPosition(1);
}
public void unhookHatchPanel(){
  hookServo.setPosition(0);
}

public static void setSpeed(VictorSPX victor, double speed){
  victor.set(ControlMode.PercentOutput, speed);
}

public static void setPosition(DoubleSolenoid doubleSolenoid, double value)
{
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

public static void setServo(Servo servo, double position){
  if (position==1.0){
    servo.setPosition(1);
  }
  if(position==0.0){
    servo.setPosition(0);
  }
}

  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new IntakeC());
  }
}