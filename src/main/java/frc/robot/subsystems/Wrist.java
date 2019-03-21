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
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Add your docs here.
 */
public class Wrist extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  public TalonSRX wristMotor;
  public AnalogInput potentiometerWrist;
  public double P = 1.6;
  public double I = 0;
  public double D = 0;
  public double error, PID, derivative, setpoint;
  public static double flat;
  public static double angle;
  public static double conversion;
  public static double armConversion;
  public static double prevArmAngle;
  public int integral, previous_error;
  public static double tooFast = .8;

  public Wrist() {
    wristMotor = new TalonSRX(RobotMap.wristMotor);
    potentiometerWrist = new AnalogInput(RobotMap.potW);


  }
  


  public void moveWrist(double value) { 
    wristMotor.set(ControlMode.PercentOutput, value);
  }
  public void movePIDSetpoint(double stick){
    setpoint -= stick * .015;

  }

  public void moveByArm(double deltaArmAngle){
    setpoint += deltaArmAngle;
  }

  public void PID(){
    angle = voltToRadians(potentiometerWrist.getAverageVoltage());
    error = setpoint - angle; // Error = Target - Actual    
    PID = P*error;
    if(PID > tooFast)
      PID = tooFast;
    if(PID < -tooFast)
      PID = -tooFast;
  }

  public double voltToRadians(double potentiometerValue){
    return (potentiometerValue - flat) * conversion;
  }

  public static void setSpeed(TalonSRX talon, double speed){
    talon.set(ControlMode.PercentOutput,speed);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    setDefaultCommand(new MoveWrist());
  }
   
}
