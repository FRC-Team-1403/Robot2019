/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
  // For example to map the left and right motors, you could define the
  // following variables to use with your drivetrain subsystem.
  // public static int leftMotor = 1;
  // public static int rightMotor = 2;
  public static int frontLeftMotor = 1;
  public static int frontRightMotor = 6;
  public static int backLeftMotor = 0;
  public static int backRightMotor = 7;
  public static int armMotorL = 2;
  public static int armMotorR = 4;

  public static int intakeMotor = 3;
  public static int wristMotor = 5;
  public static int hookServo = 1;
  
  public static int djoy = 0;
  public static int ojoy = 1;
  public static int tjoy = 2;

  public static int extensionSolenoid1 = 4;
  public static int extensionSolenoid2 = 6;
  public static int hatchPush1 = 5;
  public static int hatchPush2 = 7;
  public static int lifter1 = 2;
  public static int lifter2 = 3;


  public static int ojoyA = 1;
  public static int ojoyB = 2;
  public static int ojoyX = 3;
  public static int ojoyY = 4;
  public static int ojoyLB = 5;
  public static int ojoyRB = 6;
  public static int ojoyBack = 7;
  public static int ojoyStart = 8;
  public static int ojoyRY = 5;
  public static int ojoyLY = 1;

  public static int potA = 0;
  public static int potW = 1;

}
