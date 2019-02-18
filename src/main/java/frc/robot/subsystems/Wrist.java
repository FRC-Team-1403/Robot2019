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
  public double previousArmAngle;
  public double P = 1;
public double I = 0;
public double D = 0;
public int integral, previous_error;
public double error, PID, derivative, setpoint;
public double flat = 2.917286317999828;
public double angle;
public double conversion = .5;
public double armConversion = -.949;
public double prevArmAngle;
  public Wrist() {
    wristMotor = new TalonSRX(RobotMap.wristMotor);
    potentiometerWrist = new AnalogInput(RobotMap.potW);
    conversion = -0.9538999999999994;
    setpoint = (potentiometerWrist.getAverageVoltage()-flat)*conversion;
  }
  


  public void wristTest(double value) {
    if(potentiometerWrist.getAverageVoltage() > .39)
      wristMotor.set(ControlMode.PercentOutput, value);
    
    else
      wristMotor.set(ControlMode.PercentOutput, -.3);
  }
  public void moveBy(double stick){
    setpoint += stick * .01;
  }
  public void moveByArm(double armAngle){
    SmartDashboard.putNumber("change in wrist angle: ", armConversion * (armAngle-prevArmAngle));
    setpoint += armConversion * (armAngle-prevArmAngle);
    prevArmAngle = armAngle;
  }

  public void PID(){
    angle = voltToRadians(potentiometerWrist.getAverageVoltage());
    error = setpoint - angle; // Error = Target - Actual    
    PID = P*error;
  }

  public double voltToRadians(double potentiometerValue){
    return (potentiometerValue-Robot.w.flat)*conversion;
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
