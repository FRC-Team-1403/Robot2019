/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.commands.LiftRobot;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import frc.robot.RobotMap;
import frc.robot.Robot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Add your docs here.
 */
public class MoveLiftingPiston extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  public boolean isExtended = false;

  public DoubleSolenoid lifter;
  public DigitalOutput led;

  public MoveLiftingPiston(){
    lifter = new DoubleSolenoid(RobotMap.lifter1, RobotMap.lifter2);
    led = new DigitalOutput(0);
    led.set(false);
  }

  public void moveLiftingPiston(){
    if(Robot.m_oi.djoy.getRawButtonPressed(RobotMap.ojoyBack)) {
      // if(isExtended){
      //   lifter.set(DoubleSolenoid.Value.kReverse);
      //   isExtended = false;
      //   led.set(true);
      // }
      // else if(!isExtended){
        lifter.set(DoubleSolenoid.Value.kForward);
      // isExtended = true;
        led.set(false);
      //}
    }
    else if(Robot.m_oi.djoy.getRawButtonPressed(RobotMap.ojoyStart)){
      lifter.set(DoubleSolenoid.Value.kReverse);
      led.set(true);
    }
    else{
      lifter.set(DoubleSolenoid.Value.kOff);
    }
    
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    setDefaultCommand(new LiftRobot());
  }
}
