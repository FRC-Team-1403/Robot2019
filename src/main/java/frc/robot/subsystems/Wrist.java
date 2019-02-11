/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.command.PIDSubsystem;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import frc.robot.RobotMap;
import frc.robot.commands.MoveWrist;
import frc.robot.Robot;
/**
 * Add your docs here.
 */
public class Wrist extends PIDSubsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  public TalonSRX wristMotor;
  public AnalogInput potentiometerWrist;

  public Wrist()
  {
    super("Wrist",2.0,0.0,0.0);
    setAbsoluteTolerance(0.05);
    getPIDController().setContinuous(false);
    wristMotor = new TalonSRX(RobotMap.wristMotor);
    potentiometerWrist = new AnalogInput(RobotMap.potW);
  }
  public double returnPIDInput()
  {
    return potentiometerWrist.getAverageVoltage();
  }
  public void usePIDOutput(double output)
  {
    //wristMotor.pidWrite(output);
  }
  public void usePIDOuputVelocity(double output)
  {
    //wristMotor.pidWrite(output);
  }
  public void wristTest()
  {
    wristMotor.set(ControlMode.PercentOutput, -.5*Robot.m_oi.ojoy.getRawAxis(1));
  }
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    setDefaultCommand(new MoveWrist());
  }
   
}
