/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Servo;
import frc.robot.RobotMap;
import frc.robot.commands.HatchIntake;
/**
 * Add your docs here.
 */
public class Hatch extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  public DoubleSolenoid hatchPushSolenoid;
  public Servo hookServo;
  public Hatch()
  {
    hatchPushSolenoid = new DoubleSolenoid(RobotMap.hatchPush1, RobotMap.hatchPush2);
    hookServo = new Servo(RobotMap.hookServo);
  }

  public void push(){
    hatchPushSolenoid.set(DoubleSolenoid.Value.kForward);
  }
  public void release(){
    hatchPushSolenoid.set(DoubleSolenoid.Value.kReverse);
  }
  public void off(){
    hatchPushSolenoid.set(DoubleSolenoid.Value.kOff);
  }
  
  public void moveServo(){
    hookServo.setPosition((hookServo.getPosition()+1)%2);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    setDefaultCommand(new HatchIntake());
  }
}
