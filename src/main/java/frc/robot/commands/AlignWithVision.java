/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import edu.wpi.first.networktables.NetworkTableInstance;

public class AlignWithVision extends Command {
  static NetworkTableInstance table;
  boolean isCurrentlyOnLeft = false;
  boolean initiallyOnLeft = false;
  final double motorSpeedConst = .1;

  public static double getV(){
    return table.getDefault().getTable("limelight").getEntry("tv").getDouble(0.0);
}

  public static double getX(){
    return table.getDefault().getTable("limelight").getEntry("tx").getDouble(0.0);

}
public static double getA(){
  return table.getDefault().getTable("limelight").getEntry("ta").getDouble(0.0);
}

  public AlignWithVision() {
    // Use requires() here to declare subsystem dependencies
    requires(Robot.vs);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }
  
  public void align(double x){
    if(x < 0){
      isCurrentlyOnLeft = true;
      Robot.drivetrain.setRaw(.55, -.55); //idk what values should go here, it's just some constant
    }

    else if(x >= 0){
      isCurrentlyOnLeft = false;     
       Robot.drivetrain.setRaw(-.55, .55);
    }

  }

  double skewmoid(double area) {
		return Math.pow((1/(1 + Math.pow(Math.E, -area/4))),2);
	}

  double getMotorSpeedWithVision(double area, double angle, boolean isLeftMotor){
    final double scale = 27;
    final double motorSpeedConst = .5;
    if (isLeftMotor && angle < 0) {
        return Math.pow(Math.abs(angle/scale), 2) * skewmoid(area) * motorSpeedConst;
    } else if (isLeftMotor && angle >= 0) {
      return 0;
    } else if (!isLeftMotor && angle < 0) {
      return 0;
    } else {
      return Math.pow(Math.abs(angle/scale), 1/2) * skewmoid(area) * motorSpeedConst;
    }
  }
  
  void driveWithVision(double x){
    final double motorConstant = 0.5;
    double leftSpeed = motorConstant + getMotorSpeedWithVision(getA(), getX(), true);
    double rightSpeed = motorConstant + getMotorSpeedWithVision(getA(), getX(), false);
    Robot.drivetrain.setRaw(leftSpeed, rightSpeed);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if(Robot.m_oi.djoy.getRawButtonPressed(3)){ //should only run once, if it runs twice this is a failure
      if((int)getV() == 1){
        double x = getX();
        if(x < 0){
          isCurrentlyOnLeft = true;
          initiallyOnLeft = true;
        }
    
        else if(x >= 0){
          isCurrentlyOnLeft = false;
          initiallyOnLeft = false;
        }
      }
    } 
    if(Robot.m_oi.djoy.getRawButton(3)){
      if((int)getV() == 1){
        double x = getX();
        if(isCurrentlyOnLeft == initiallyOnLeft)
            align(x);
        else if(getA() >= 10){
          Robot.drivetrain.stop();
        }
        else{
          driveWithVision(x);
        }
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
