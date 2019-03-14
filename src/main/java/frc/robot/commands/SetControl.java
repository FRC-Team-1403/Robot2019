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
  public static final Setpoint[] hatchPositions = {new Setpoint(-0.6579435393, 0.7508715262,true), new Setpoint(-0.6521623529, 2.31684621524), new Setpoint(0.1979114756, 1.439048824), new Setpoint(0.9303370598, 0.7683379551, true)};
  public static final Setpoint[] ballPositions = {new Setpoint(-0.6936229335634126, .2364825), new Setpoint(-0.6521623529, 2.31684621524), new Setpoint(-0.2469002464, 0.3933919486), new Setpoint(.390508, -.205), new Setpoint(1.011, -0.687, true)};//new Setpoint(youneedtodocargoarm, youneedtodocargowrist), <- this goes third to last
  //hatch pickup, 1, 2, 3
  //ball pickup, HPS, 1, (not implemented)cargo, 2, 3
   
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
    Robot.rioIO.writeToRIO(aInit, wInit, aConversion, wConversion);
  }

  public void checkCallibration(){
    if(Robot.m_oi.tjoy.getRawButtonPressed(RobotMap.ojoyStart)) {
      SmartDashboard.putBoolean("NIVA START HAS BEEN PRESSED: ", Robot.m_oi.tjoy.getRawButtonPressed(RobotMap.ojoyStart));
      double startArmReading = Robot.arm.potentiometerArm.getAverageVoltage();
      double startWristReading = Robot.w.potentiometerWrist.getAverageVoltage();
      while(true){
        if(Robot.m_oi.tjoy.getRawButtonPressed(RobotMap.ojoyLB)){
          break;
        }
        if(Robot.m_oi.tjoy.getRawButtonPressed(RobotMap.ojoyRB)){
            updatePotentiometerReadings(startArmReading, startWristReading, Robot.arm.potentiometerArm.getAverageVoltage(), Robot.w.potentiometerWrist.getAverageVoltage());
            break;
          }
        }
      } 
  }
  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    
    if(Robot.m_oi.ojoy.getRawButtonPressed(RobotMap.ojoyLB))
    {
      if(Robot.m_oi.ojoy.getRawButton(RobotMap.ojoyStart)) {
        /*mode++;
        mode%=2;*/
        //cargo ball
      }
      else if(Robot.m_oi.ojoy.getRawButton(RobotMap.ojoyX))
      {
        //floor ball
        ballPositions[0].run();
      }
      else if(Robot.m_oi.ojoy.getRawButton(RobotMap.ojoyA))
      {
        //level 1 ball
        ballPositions[2].run();
      }
      else if(Robot.m_oi.ojoy.getRawButton(RobotMap.ojoyB))
      {
        //level 2 ball
        ballPositions[3].run();
      }
      else if(Robot.m_oi.ojoy.getRawButton(RobotMap.ojoyY))
      {
        //level 3 ball
        ballPositions[4].run();
      }
    }
    else
    {
      checkCallibration();
      if(Robot.m_oi.ojoy.getRawButton(RobotMap.ojoyX))
      {
        //floor hatch
        hatchPositions[0].run();
      }
      else if(Robot.m_oi.ojoy.getRawButton(RobotMap.ojoyA))
      {
        //level 1 hatch
        hatchPositions[1].run();
      }
      else if(Robot.m_oi.ojoy.getRawButton(RobotMap.ojoyB))
      {
        //level 2 hatch
        hatchPositions[2].run();
      }
      else if(Robot.m_oi.ojoy.getRawButton(RobotMap.ojoyY))
      {
        //level 3 hatch
        hatchPositions[3].run();
      }
    }
    
    // else
    // checkCallibration();
    //   if(joystickMoved() && !Robot.m_oi.ojoy.getRawButton(RobotMap.ojoyX)) {
     
    //     if(Robot.m_oi.ojoy.getRawAxis(RobotMap.ojoyLY) < -.5) {
    //       if(mode == 1) {
    //        ballLevel++;        
    //        if(ballLevel == ballPositions.length) {
    //          ballLevel = ballPositions.length-1;
    //        }
    //        ballPositions[ballLevel].run();

    //      }
    //      if(mode == 0) {
    //        hatchLevel++;
    //        if(hatchLevel == hatchPositions.length) {
    //          hatchLevel = hatchPositions.length-1;
    //         }
    //        hatchPositions[hatchLevel].run(); 
    //      }
    //    } else {
    //      if(mode == 1) {
    //        ballLevel--;
    //         if(ballLevel == -1){
    //          ballLevel = 0;
    //        }
    //        ballPositions[ballLevel].run(); 
    //      }
    //      if(mode == 0) {
    //        hatchLevel--;
    //        if(hatchLevel == -1) {
    //          hatchLevel = 0;
    //        }
    //        hatchPositions[hatchLevel].run(); 
    //     }
    //   }
    // }
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
