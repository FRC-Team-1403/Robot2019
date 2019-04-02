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
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Setpoint;
import frc.robot.commands.SetControl;

/**
 * Add your docs here.
 */
public class Arm extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
public TalonSRX armMotorL;
public TalonSRX armMotorR;
public AnalogInput potentiometerArm;

//.75087152622, 0.39339194857, 2.21688712317, 1.43904882392, 0.76833795509
// 0.38407651983, 0.39339194857, -0.50554692422, -0.83158692989

public static double feedForwardConstant = 0; //to find how to deal with torque
public double armConstant = .1; //to control speed at which joystick moves arm
public double ratio = 1; //ratio to convert from voltage of potentiometer to radians
public double P = 4.030000000000104;
public double I = 0.08000000000000006;
public double D = 0.03799999999999987;
public double integral, previous_error;
public double error, PID, derivative, setpoint;
public static double flat;
public static double angle;
public static double conversion;
public final double tooFast = 10000;

public Arm()
  {
    armMotorL = new TalonSRX(RobotMap.armMotorL);
    armMotorR = new TalonSRX(RobotMap.armMotorR);
    potentiometerArm = new AnalogInput(RobotMap.potA);
    conversion = -1.032240000077177;
    
  }

  public void moveArm(double input) {
    armMotorL.set(ControlMode.PercentOutput, -input);
    armMotorR.set(ControlMode.PercentOutput, input);
  }

  public void PID(){
    angle = voltToRadians(potentiometerArm.getAverageVoltage());
    error = setpoint - angle; // Error = Target - Actual
    this.integral += (error*.02); // Integral is increased by the error*time (which is .02 seconds using normal IterativeRobot)
    derivative = (error - this.previous_error) / .02;
    PID = P*error + this.integral * I + derivative * D;
    /*if(PID > tooFast)
      PID = tooFast;
    if(PID < -tooFast)
      PID = -tooFast;*/
    this.previous_error = this.error;
  }

  public void moveBy(double stick) {
    if(Math.abs(stick) < .05){
      return;
    }
    setpoint += -stick * 0.0075;
  }

  public static double feedForward(double angle) {
    return feedForwardConstant * Math.cos(angle);
  }
  
  public static double voltToRadians(double potentiometerValue)
  {
    return (potentiometerValue - flat)*conversion;
  }

  public static double radiansToVolts(double radians)
  {
    return radians/conversion + flat;
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
