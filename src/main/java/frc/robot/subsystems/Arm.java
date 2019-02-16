/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Servo;
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


public final double floorAngle = 0;
public final double hpAngle = 0;
public final double firstAngle = 0;
public final double secondAngle = 0;
public final double thirdAngle = 0;
public double feedForwardConstant = 0; //to find how to deal with torque
public double armConstant = .1; //to control speed at which joystick moves arm
public double ratio = ; //ratio to convert from voltage of potentiometer to radians

  public Arm()
  {
    super("Arm",2.0,0.0,0.0);
    setAbsoluteTolerance(0.05);
    getPIDController().setContinuous(false);
    armMotorL = new TalonSRX(RobotMap.armMotorL);
    armMotorR = new TalonSRX(RobotMap.armMotorR);
    potentiometerArm = new AnalogInput(RobotMap.potA);
  }
  public double returnPIDInput(){
    return potentiometerArm.getAverageVoltage() * (ratio);
  }
  public void usePIDOutput(double output) {
    armMotorL.set(ControlMode.Position, -output - feedForwardConstant*Math.cos(output));
    armMotorR.set(ControlMode.Position, output + feedForwardConstant * Math.cos(output));
  }
  public void usePIDOuputVelocity(double output){
    //armMotorL.pidWrite(output);
  }
  public void armTest() {

    armMotorL.set(ControlMode.Position, -Robot.m_oi.ojoy.getRawAxis(RobotMap.ojoyLY)*armConstant - potentiometerArm.getAverageVoltage() * feedForwardConstant * ratio);
    armMotorR.set(ControlMode.Position, Robot.m_oi.ojoy.getRawAxis(RobotMap.ojoyLY)*armConstant + potentiometerArm.getAverageVoltage() * feedForwardConstant * ratio);
  }

  public void moveWithPID(){
    setSetpoint(getPosition()+Robot.m_oi.ojoy.getRawAxis(RobotMap.ojoyLY));
  }

  public static void setSpeed(TalonSRX talon, double speed){
    talon.set(ControlMode.PercentOutput, speed);
  }
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    setDefaultCommand(new MoveArm());
  }
  
}
