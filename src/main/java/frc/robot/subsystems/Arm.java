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
/**
 * Add your docs here.
 */
public class Arm extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
public TalonSRX armMotorL;
public TalonSRX armMotorR;
public AnalogInput potentiometerArm;


public final Setpoint[] hatchPositions = {new Setpoint(0,0), new Setpoint(4.353026898, 2.130126735), new Setpoint(4.401855018, 0.5932616580000001), new Setpoint(3.455810193, 1.408691262), new Setpoint(2.6879880060000003, 2.1118161900000003)};
public final Setpoint[] ballPositions = {new Setpoint(0,0), new Setpoint(4.210204647, 2.51464818), new Setpoint(3.922118739, 2.504882556), new Setpoint(2.935790715, 3.447265272), new Setpoint(2.302245858, 3.7890621120000003)};
public static double feedForwardConstant = 0; //to find how to deal with torque
public double armConstant = .1; //to control speed at which joystick moves arm
public double ratio = 1; //ratio to convert from voltage of potentiometer to radians
public double P = 4.604;
public double I = 0;
public double D = 0;
public int integral, previous_error;
public double error, PID, derivative, setpoint;
public double flat = 3.663286317999746;
public double angle;
public static double conversion;
public final double tooFast = .5;

  public Arm()
  {
    armMotorL = new TalonSRX(RobotMap.armMotorL);
    armMotorR = new TalonSRX(RobotMap.armMotorR);
    potentiometerArm = new AnalogInput(RobotMap.potA);
    conversion = -0.9538999999999994;
    
    angle = (potentiometerArm.getAverageVoltage()-flat)*conversion;
    setpoint = angle;
    
  }

  public void armTest(double input) {
    armMotorL.set(ControlMode.PercentOutput, -input);
    armMotorR.set(ControlMode.PercentOutput, input);
  }

  public void PID(){
    angle = voltToRadians(potentiometerArm.getAverageVoltage());
    error = setpoint - angle; // Error = Target - Actual
    this.integral += (error*.02); // Integral is increased by the error*time (which is .02 seconds using normal IterativeRobot)
    derivative = (error - this.previous_error) / .02;
    PID = P*error + feedForward(angle);
    if(PID > tooFast)
      PID = tooFast;
    if(PID < -tooFast)
      PID = -tooFast;
  }

  public void moveBy(double stick) {
    if(Math.abs(stick) < .05){
      return;
    }
      setpoint += -stick * 0.01;
  }

  public static double feedForward(double angle) {
    return Arm.feedForwardConstant*Math.cos(angle);
  }
  
  public static double voltToRadians(double potentiometerValue)
  {
    return (potentiometerValue-Robot.arm.flat)*conversion;
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
