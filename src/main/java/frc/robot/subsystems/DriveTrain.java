/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import frc.robot.DriveSignal;
import frc.robot.CheesyDriveHelper;
import frc.robot.RobotMap;
import frc.robot.commands.DriveWithJoystick;
import frc.robot.Robot;

/**
 * Add your docs here.
 */
public class DriveTrain extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
public TalonSRX frontLeft;
public TalonSRX frontRight;
public TalonSRX backLeft;
public TalonSRX backRight;
public CheesyDriveHelper helper;


  public DriveTrain()
  {
    frontLeft = new TalonSRX(RobotMap.frontLeftMotor);
    frontRight = new TalonSRX(RobotMap.frontRightMotor);
    backLeft = new TalonSRX(RobotMap.backLeftMotor);
    backRight = new TalonSRX(RobotMap.backRightMotor);
    helper = new CheesyDriveHelper();
    setMaxAccerlation(Robot.DEFAULTMAXACCELERATION);
  }
  public void setMaxAccerlation(double secondsFromNeutralToFull) {
    frontLeft.configClosedloopRamp(secondsFromNeutralToFull);
    backLeft.configClosedloopRamp(secondsFromNeutralToFull);
    frontRight.configClosedloopRamp(secondsFromNeutralToFull);
    backRight.configClosedloopRamp(secondsFromNeutralToFull);
  }

  public void setRaw(double leftValue, double rightValue) {
    backLeft.set(ControlMode.Velocity, leftValue * Robot.maxRPM * 4096 / 600);
    backRight.set(ControlMode.Velocity, -rightValue * Robot.maxRPM * 4096 / 600);
  }
  public void turnLeft() {
    setRaw(-0.55,0.55);
  }

  public void turnRight() {
    setRaw(0.55, -0.55);
  }

  public void moveForward() {
    setRaw(0.75, 0.75);
  }

  public void moveBackward(){
    setRaw(-0.4, -0.4);
  }

  public void tiltRight(){
    setRaw(0.5, 0.45);
  }

  public void tiltLeft(){
    setRaw(0.45, 0.5);
  }
  public void stop(){
    setRaw(0,0);
  }
  public void driveCheesy()
  {
    DriveSignal signal = helper.cheesyDrive(-Robot.m_oi.djoy.getRawAxis(1), Robot.m_oi.djoy.getRawAxis(4), Robot.m_oi.djoy.getRawButton(6));
    backLeft.set(ControlMode.Velocity, .5*signal.mLeftMotor * Robot.maxRPM * 4096 / 600);
    backRight.set(ControlMode.Velocity, -.5*signal.mRightMotor * Robot.maxRPM * 4096 / 600);
  }
  
  public void driveTank()
  {
    backLeft.set(ControlMode.Velocity, -.5 * Robot.m_oi.djoy.getRawAxis(1) * Robot.maxRPM * 4096 / 600);
    backRight.set(ControlMode.Velocity, .5 * Robot.m_oi.djoy.getRawAxis(5) * Robot.maxRPM * 4096 / 600);
  }
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    setDefaultCommand(new DriveWithJoystick());
  }
}
