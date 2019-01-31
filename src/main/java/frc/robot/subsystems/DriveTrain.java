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
import frc.robot.DriveSignal;
import frc.robot.CougarDriveHelper;
import frc.robot.RobotMap;
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
CougarDriveHelper helper;
  public DriveTrain()
  {
    frontLeft = new TalonSRX(RobotMap.frontLeftMotor);
    frontRight = new TalonSRX(RobotMap.frontRightMotor);
    backLeft = new TalonSRX(RobotMap.backLeftMotor);
    backRight = new TalonSRX(RobotMap.backRightMotor);
  }
  public void drive()
  {
    DriveSignal signal = helper.cheesyDrive(-Robot.m_oi.djoy.getRawAxis(1), Robot.m_oi.djoy.getRawAxis(4), Robot.m_oi.djoy.getRawButton(6));
    backLeft.set(ControlMode.Velocity, signal.mLeftMotor * Robot.maxRPM * 4096 / 600);
    backRight.set(ControlMode.Velocity, -signal.mRightMotor * Robot.maxRPM * 4096 / 600);
  }
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
