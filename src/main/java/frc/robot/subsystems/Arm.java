/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.command.PIDSubsystem;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.MoveArm;
import com.ctre.phoenix.motorcontrol.ControlMode;

/**
 * Add your docs here.
 */
public class Arm extends PIDSubsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
public TalonSRX armMotorL;
public TalonSRX armMotorR;
public AnalogInput potentiometerArm;
  public Arm()
  {
    super("Arm",2.0,0.0,0.0);
    setAbsoluteTolerance(0.05);
    getPIDController().setContinuous(false);
    armMotorL = new TalonSRX(RobotMap.armMotorL);
    armMotorR = new TalonSRX(RobotMap.armMotorR);
    potentiometerArm = new AnalogInput(RobotMap.potA);
  }
  public double returnPIDInput()
  {
    return potentiometerArm.getAverageVoltage();
  }
  public void usePIDOutput(double output)
  {
    //armMotorL.pidWrite(output);
  }
  public void usePIDOuputVelocity(double output)
  {
    //armMotorL.pidWrite(output);
  }
  public void armTest()
  {
    armMotorL.set(ControlMode.PercentOutput, -Robot.m_oi.ojoy.getRawAxis(5));
    armMotorR.set(ControlMode.PercentOutput, Robot.m_oi.ojoy.getRawAxis(5));

  }
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    setDefaultCommand(new MoveArm());
  }
  
}
