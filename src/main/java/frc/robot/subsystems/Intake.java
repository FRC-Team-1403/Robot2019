/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.RobotMap;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Robot;

/**
 * Add your docs here.
 */
public class Intake extends Subsystem {
  public CANSparkMax intakeMotor;

  public Intake(){
    intakeMotor = new CANSparkMax(RobotMap.intakeMotor, MotorType.kBrushless);
  }
public void intake(){
  if(Robot.m_oi.ojoy.getRawAxis(2)>0)
  {
    intakeMotor.set(Robot.m_oi.ojoy.getRawAxis(2));
  }
  else if(-Robot.m_oi.ojoy.getRawAxis(3)<0)
  {
    intakeMotor.set(-Robot.m_oi.ojoy.getRawAxis(3));
  }
  else
  {
    intakeMotor.set(0);
  }
}

  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
