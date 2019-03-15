/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotMap;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
public class AlignWithVision extends Command {
  public static NetworkTableInstance table;
  public boolean isCurrentlyOnLeft = false;
  public boolean initiallyOnLeft = false;
  public double area = 0;
  public double motorSpeedConst = .2;
  public double turnSpeed = .4;
  double prevArmSetpoint;
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
    requires(Robot.vs);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }
  
  public void align(double x){
    if(x < 0){
      isCurrentlyOnLeft = true;
      Robot.drivetrain.setRaw(-1*turnSpeed, turnSpeed); 
    }

    else if(x >= 0){
      isCurrentlyOnLeft = false;     
       Robot.drivetrain.setRaw(turnSpeed, -turnSpeed);
    }

  }

  public double skewmoid(double area) {
    return 2.0/3*1/(1 + Math.pow(Math.E, -2*area+2)) + 1.0/3;
	}

  public double getMotorSpeedWithVision(double area, double angle, boolean isLeftMotor){
    final double scale = 13;
    double dconst = 0.8;
    this.area = area;
    SmartDashboard.putNumber("area: ", area);
    if (isLeftMotor && angle < 0) {
        return Math.pow(Math.abs(angle/scale), 0.333) * skewmoid(area) * dconst;
    } else if (isLeftMotor && angle >= 0) {
      return 0;
    } else if (!isLeftMotor && angle < 0) {
      return 0;
    } else {
      return Math.pow(Math.abs(angle/scale), 0.333) * skewmoid(area) * dconst;
    }
  }
  
  public void driveWithVision(){
    this.motorSpeedConst = turnSpeed * Math.pow(-(this.area/10.0) + 1.0, 0.5);
    double leftSpeed = motorSpeedConst + getMotorSpeedWithVision(getA(), -getX(), true);
    double rightSpeed = motorSpeedConst + getMotorSpeedWithVision(getA(), -getX(), false);
    SmartDashboard.putNumber("left speed: ", + leftSpeed); 
    SmartDashboard.putNumber("right speed: ", + rightSpeed);
    SmartDashboard.putNumber("limelight x: ", + getX());
    SmartDashboard.putNumber("motor constant: ", + this.motorSpeedConst);
    Robot.drivetrain.setRaw(leftSpeed, rightSpeed);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if(Robot.m_oi.djoy.getRawButtonPressed(RobotMap.ojoyX)){
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
        //prevArmSetpoint = Robot.arm.setpoint;
        //SetControl.ballPositions[1].run();
      }
    
    } 
    if(Robot.m_oi.djoy.getRawButton(RobotMap.ojoyX)){
      if((int)getV() == 1){
        double x = getX();
        if(isCurrentlyOnLeft == initiallyOnLeft)
            align(x);
        else{
          driveWithVision();
        }
      }
    }
    if(Robot.m_oi.djoy.getRawButtonReleased(RobotMap.ojoyX)){
      Robot.drivetrain.stop();
      //Robot.arm.setpoint = prevArmSetpoint;
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
