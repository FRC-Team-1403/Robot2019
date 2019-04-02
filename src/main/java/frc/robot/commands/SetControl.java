/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.Setpoint;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SetControl extends Command {
  public static int ballLevel = 0; // 0 - pick up, 1 - HPS, 2 - first level, 3 - second level, 4 - third level
  public static int hatchLevel = 0; // 0 - pick up, 1 - first level, 2 - second level, 3 - third level
  public static int mode = 0; //0 - hatch control, 1 - ball control
  boolean currentlyPressed = false;
  public static final double armCallibrationAngle = -50.2;
  public static final double wristCallibrationAngle = 137;
  //angles are relative to the flat
  public static final Setpoint[] hatchPositions = {new Setpoint(-0.6579435393, 0.7508715262,true, false), new Setpoint(-0.6703075464933462, 2.3430167757303044), new Setpoint(0.1979114756, 1.439048824), new Setpoint(0.6758309715586512, 0.8599574042501887, true, false)};
  public static final Setpoint[] ballPositions = {new Setpoint(-0.766999725800844, 0.4023758239272958, false, true), new Setpoint(-0.2901915591978343, 0.6042493691922602), new Setpoint(0.469908753990639, -0.5179280307362237), new Setpoint(0.469908753990639, -0.060424936919227124), new Setpoint(0.9351982398947328, -0.7089404942355213, true, false)};//new Setpoint(youneedtodocargoarm, youneedtodocargowrist), <- this goes third to last
  boolean shouldUpdate = true;
  double startArmReading = 0; 
  double startWristReading = 0; 
  //hatch pickup, 1, 2, 3
  //ball pickup, 1, cargo, 2, 3
   
  public SetControl() {
    requires(Robot.cs);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    //read armPot and wristPot from file or set to default values
  }
  
  boolean joystickMoved() {
    double stick = Robot.m_oi.ojoy.getRawAxis(RobotMap.ojoyLY);
    if(currentlyPressed && Math.abs(stick) > .5) {
      return false;
    } else if(currentlyPressed && Math.abs(stick) < .5) {
      currentlyPressed = false;
      return false;
    } else if(!currentlyPressed && Math.abs(stick) > .5) {
      currentlyPressed = true;
      return true;
    }
    return false;
  }

  public void updatePotentiometerReadings(double aInit, double wInit, double aFinal, double wFinal){
    double aConversion = -1 * Math.abs(Math.PI/180.00 * (armCallibrationAngle)/(aFinal-aInit));
    double wConversion = -1 * Math.abs(Math.PI/180.00 * (wristCallibrationAngle)/(wFinal - wInit)); 
    SmartDashboard.putNumber("arm Conversion", aConversion);
    SmartDashboard.putNumber("wrist Conversion", wConversion);
    Robot.rioIO.writeToRIO(aInit, wInit, aConversion, wConversion);
  }

  public void checkCallibration(){
   if(Robot.m_oi.tjoy.getRawButtonPressed(RobotMap.ojoyStart)) {
    SmartDashboard.putBoolean("reached callibration", shouldUpdate); 
    
    if(shouldUpdate){
      startArmReading = Robot.arm.potentiometerArm.getAverageVoltage();
      startWristReading = Robot.w.potentiometerWrist.getAverageVoltage();
      SmartDashboard.putNumber("Init armPot", startArmReading);
      SmartDashboard.putNumber("Init wPot", startWristReading);
      shouldUpdate = false;
    }
    else{  
      updatePotentiometerReadings(startArmReading, startWristReading, Robot.arm.potentiometerArm.getAverageVoltage(), Robot.w.potentiometerWrist.getAverageVoltage());
      SmartDashboard.putNumber("Final armPot", Robot.arm.potentiometerArm.getAverageVoltage());
      SmartDashboard.putNumber("Final wPot", Robot.w.potentiometerWrist.getAverageVoltage());
      shouldUpdate = true;
      }
    }
  }
  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    checkCallibration();
    if(Robot.m_oi.ojoy.getRawButton(RobotMap.ojoyLB))
    {
      if(Robot.m_oi.ojoy.getRawButton(RobotMap.ojoyStart)) {
        //cargo
        ballPositions[2].run();
      }
      else if(Robot.m_oi.ojoy.getRawButtonPressed(RobotMap.ojoyX))
      {
        //floor ball
        ballPositions[0].run();
      }
      else if(Robot.m_oi.ojoy.getRawButtonPressed(RobotMap.ojoyA))
      {
        //level 1 ball
        ballPositions[1].run();
      }
      else if(Robot.m_oi.ojoy.getRawButtonPressed(RobotMap.ojoyB))
      {
        //level 2 ball
        ballPositions[3].run();
      }
      else if(Robot.m_oi.ojoy.getRawButtonPressed(RobotMap.ojoyY))
      {
        //level 3 ball
        ballPositions[4].run();
      }
    }
    else
    {
      if(Robot.m_oi.ojoy.getRawButtonPressed(RobotMap.ojoyX))
      {
        //floor hatch
        hatchPositions[0].run();
      }
      else if(Robot.m_oi.ojoy.getRawButtonPressed(RobotMap.ojoyA))
      {
        //level 1 hatch
        hatchPositions[1].run();
      }
      else if(Robot.m_oi.ojoy.getRawButtonPressed(RobotMap.ojoyB))
      {
        //level 2 hatch
        hatchPositions[2].run();
      }
      else if(Robot.m_oi.ojoy.getRawButtonPressed(RobotMap.ojoyY))
      {
        //level 3 hatch
        hatchPositions[3].run();
      }
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
