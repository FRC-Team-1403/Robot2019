/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import com.revrobotics.*;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.RobotMap;
import frc.robot.Robot;

/**
 * Add your docs here.
 */
public class DriveTrain extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
CANSparkMax frontLeft;
CANSparkMax frontRight;
CANSparkMax backLeft;
CANSparkMax backRight;
  public DriveTrain()
  {
    frontLeft = new CANSparkMax(RobotMap.frontLeftMotor, MotorType.kBrushless);
    frontRight = new CANSparkMax(RobotMap.frontRightMotor, MotorType.kBrushless);
    backLeft = new CANSparkMax(RobotMap.backLeftMotor, MotorType.kBrushless);
    backRight = new CANSparkMax(RobotMap.backRightMotor, MotorType.kBrushless);
  }
  public void drive()
  {
    frontLeft.set(Robot.m_oi.djoy.getRawAxis(1));
    backLeft.set(Robot.m_oi.djoy.getRawAxis(1));
    frontRight.set(Robot.m_oi.djoy.getRawAxis(5));
    backRight.set(Robot.m_oi.djoy.getRawAxis(5));
  }
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
