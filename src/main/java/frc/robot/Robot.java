/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.SetControl;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Hatch;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.ArmExtension;
import frc.robot.subsystems.ControlSystem;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Wrist;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.subsystems.MoveLiftingPiston;
import com.ctre.phoenix.motorcontrol.ControlMode;
import frc.robot.subsystems.Vision;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import frc.robot.echo.Recorder;



/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  public static OI m_oi;
  public static DriveTrain drivetrain;
  public static Intake in;
  public static ArmExtension ae;
  public static Arm arm;
  public static Hatch hatch;
  public static Wrist w;
  public static ControlSystem cs;
  public static MoveLiftingPiston lift;
  public static Vision vs;

  public static double FASTMAXRPM = 500;
  public static double DEFAULTMAXRPM = 600;
  public static double FINECONTROLRPM = 100;
  public static double maxRPM = DEFAULTMAXRPM;

  public static double FINECONTROLMAXACCELERATION = 100;
  public static double DEFAULTMAXACCELERATION = 0.0002;
  public static double HYPERSPEEDMAXACCELERATION = 10000;

  public static NetworkTableInstance table;
  public static NetworkTableEntry tx;
  public static NetworkTableEntry tv;
  public static NetworkTableEntry ta;

  public static IOWithRIO rioIO;

  public static Recorder recorder;
  public static boolean record;
  public static boolean store;
  private int numpaths;
  private String path;
  Command autonomousCommand;
  String pathChooser;
  int delay;
  int autoint;

  boolean shouldReset;

  Command m_autonomousCommand;
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    numpaths = 0;
    delay = 0;
    autoint = 0;
    hatch = new Hatch();
    //recorder = new Recorder(10000);
    drivetrain = new DriveTrain();
    rioIO = new IOWithRIO();
    in = new Intake();
    ae =  new ArmExtension();
    arm = new Arm();
    w = new Wrist();
    cs = new ControlSystem();
    vs = new Vision();
    lift = new MoveLiftingPiston();
    m_oi = new OI();
    shouldReset = true;
    // chooser.addOption("My Auto", new MyAutoCommand());
    SmartDashboard.putData("Auto mode", m_chooser);
    SmartDashboard.putBoolean("USE PISTONS", hatch.hookServo.getPosition()>0.5);
    talonInitVelocity(drivetrain.frontLeft);
    talonInitVelocity(drivetrain.frontRight);
    talonInitVelocity(drivetrain.backLeft);
    talonInitVelocity(drivetrain.backRight);

    drivetrain.frontLeft.follow(drivetrain.backLeft);
    drivetrain.frontRight.follow(drivetrain.backRight);


    drivetrain.backLeft.setSensorPhase(true);
    drivetrain.backRight.setSensorPhase(true);
    
    drivetrain.frontLeft.setInverted(false); 
    drivetrain.backLeft.setInverted(false);
    drivetrain.frontRight.setInverted(false);
    drivetrain.backRight.setInverted(false);
    // in.intakeMotor.setInverted(true);
    resetPotentiometers();
    // Robot.arm.flat = 3.544921512;
    // Robot.w.flat = 2.504882556;
    // Robot.arm.conversion = -0.9357842015131478;
    // Robot.w.conversion = -3.580969289598466;
    hatch.hookServo.setPosition(1);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   * You can use it to reset any subsystem information you want to clear when
   * the robot is disabled.
   */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
    
    if (Robot.m_oi.ojoy.getRawButtonReleased(8)) { autoint++; }
		SmartDashboard.putNumber("autoint", autoint%4);
		
		if (autoint%4 == 0) { SmartDashboard.putString("Auto Position", "Left"); }
		if (autoint%4 == 1) { SmartDashboard.putString("Auto Position", "Right"); }
		if (autoint%4 == 2) { SmartDashboard.putString("Auto Position", "Middle"); }
		if (autoint%4 == 3) { SmartDashboard.putString("Auto Position", "STRAIGHT"); }
    Scheduler.getInstance().run();
    
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString code to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional commands to the
   * chooser code above (like the commented example) or additional comparisons
   * to the switch structure below with additional strings & commands.
   */

  public void resetPotentiometers(){
    
    rioIO.readFromRIO();
    arm.angle = (arm.potentiometerArm.getAverageVoltage()-arm.flat)*arm.conversion;
    arm.setpoint = arm.angle;
    w.angle = (w.potentiometerWrist.getAverageVoltage()-w.flat)*w.conversion;
    w.setpoint = w.angle;
    hatch.hatchPushSolenoid.set(DoubleSolenoid.Value.kForward);
    Robot.arm.potentiometerArm.resetAccumulator();
  }

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_chooser.getSelected();
    

    /*
     * String autoSelected = SmartDashboard.getString("Auto Selector",
     * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
     * = new MyAutoCommand(); break; case "Default Auto": default:
     * autonomousCommand = new ExampleCommand(); break; }
     */

    // schedule the autonomous command (example)
    /*if (m_autonomousCommand != null) {
      
    	recorder.setCurrentWritefile(1);
		  recorder.setCurrentReadfile(0);
		  Recorder.initWriter();
		  Recorder.initReader();
		  recorder.resetReadings();
		  recorder.storeReadings();
		
		
		
      m_autonomousCommand.start();
    }*/
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    
    // if(recorder.hasNextLine())
		// {
		// 	System.out.println(recorder.getReading("DriveTrain Back Left"));
		// 	DriveTrain.setSpeed(drivetrain.frontLeft, recorder.getReading("DriveTrain Front Left"));
		// 	DriveTrain.setSpeed(drivetrain.backLeft, recorder.getReading("DriveTrain Back Left"));
		// 	DriveTrain.setSpeed(drivetrain.frontRight, recorder.getReading("DriveTrain Front Right"));
		// 	DriveTrain.setSpeed(drivetrain.backRight, recorder.getReading("DriveTrain Back Right"));
		// 	Wrist.setSpeed(w.wristMotor, recorder.getReading("Wrist"));
    //   Intake.setSpeed(in.intakeMotor, recorder.getReading("Intake Ball"));
    //   Intake.setSpeed(in.intakeMotor, recorder.getReading("Eject Ball"));
    //   Intake.setPosition(in.hatchPushSolenoid, recorder.getReading("Push Hatch"));
    //   Intake.setPosition(in.hatchPushSolenoid, recorder.getReading("Release Hatch"));
    //   Intake.setServo(in.hookServo, recorder.getReading("Hook Hatch"));
    //   Intake.setServo(in.hookServo, recorder.getReading("Unhook Hatch"));
    //   Arm.setSpeed(arm.armMotorL, recorder.getReading("Move Arm Left"));
    //   Arm.setSpeed(arm.armMotorR, recorder.getReading("Move Arm Right"));
    //   ArmExtension.setPosition(ae.armExtender, recorder.getReading("Arm is extended"));
      
    //   ArmExtension.setPosition(ae.armExtender, recorder.getReading("Arm is retracted"));
		// 	Timer.delay(0.001);
		// }
		
		// else
		// {
    //   DriveTrain.setSpeed(drivetrain.frontLeft, 0);
		// 	DriveTrain.setSpeed(drivetrain.backLeft, 0);
		// 	DriveTrain.setSpeed(drivetrain.frontRight, 0);
		// 	DriveTrain.setSpeed(drivetrain.backRight, 0);
		// 	Wrist.setSpeed(w.wristMotor, 0);
    //   Intake.setSpeed(in.intakeMotor, 0);
    //   Intake.setSpeed(in.intakeMotor, 0);
    //   Intake.setPosition(in.hatchPushSolenoid, 0);
    //   Intake.setPosition(in.hatchPushSolenoid, 0);
    //   Intake.setServo(in.hookServo, 0);
    //   Intake.setServo(in.hookServo, 0);
    //   Arm.setSpeed(arm.armMotorL,0);
    //   Arm.setSpeed(arm.armMotorR, 0);
    //   ArmExtension.setPosition(ae.armExtender, 0);
    //   ArmExtension.setPosition(ae.armExtender, 0);
    // }
    this.teleopPeriodic();
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    FINECONTROLMAXACCELERATION = SmartDashboard.getNumber("Fine Acceleration: ", 100);
    DEFAULTMAXACCELERATION = SmartDashboard.getNumber("Default Acceleration: ", 500);
    HYPERSPEEDMAXACCELERATION = SmartDashboard.getNumber("Hyper Speed: ", 10000);

    FASTMAXRPM = SmartDashboard.getNumber("FAST Max RPM:", 8000);
    DEFAULTMAXRPM = SmartDashboard.getNumber("Default Max RPM:", 5700);
    FINECONTROLRPM = SmartDashboard.getNumber("Fine Max RPM:", 3000);
    
    SmartDashboard.putNumber("Wrist Conversion from Robot", Robot.w.conversion);
    SmartDashboard.putNumber("Arm Setpoint", Robot.arm.setpoint);
    SmartDashboard.putNumber("Wrist Angle: ", Robot.w.angle);
    SmartDashboard.putNumber("Wrist Setpoint", Robot.w.setpoint);
    SmartDashboard.putNumber("Arm Angle", Robot.arm.angle);
    SmartDashboard.putNumber("Arm Potentiometer", Robot.arm.potentiometerArm.getAverageVoltage());
    SmartDashboard.putNumber("Wrist Potentiometer", Robot.w.potentiometerWrist.getAverageVoltage());
    SmartDashboard.putBoolean("USE PISTONS", hatch.hookServo.getPosition()<0.5);
    /*if(Recorder.isRecording)
		{
			recorder.addReading("DriveTrain Back Left", -Robot.m_oi.djoy.getRawAxis(1));
			recorder.addReading("DriveTrain Back Right", Robot.m_oi.djoy.getRawAxis(5));
			recorder.addReading("DriveTrain Front Left", -Robot.m_oi.djoy.getRawAxis(1));
			recorder.addReading("DriveTrain Front Right", Robot.m_oi.djoy.getRawAxis(5));
			System.out.println(recorder.getReading("DriveTrain Back Left"));
			System.out.println(recorder.getReading("DriveTrain Back Right"));
			recorder.addReading("Wrist", Robot.m_oi.ojoy.getRawAxis(1));
      recorder.addReading("Intake Ball", Robot.m_oi.ojoy.getRawAxis(2));
      recorder.addReading("Eject Ball", Robot.m_oi.ojoy.getRawAxis(3));
      recorder.addReading("Push Hatch", Robot.m_oi.djoy.getRawAxis(5)); //boolToDouble is static in ArmExtension, it was intiailly
      recorder.addReading("Release Hatch", ArmExtension.convertBoolToDouble()); //Robot.in.convertBoolToDouble but that wasn't where it was
      recorder.addReading("Hook Hatch", ArmExtension.convertBoolToDouble()); //if this doesn't work then that is why
      recorder.addReading("Unhook Hatch", ArmExtension.convertBoolToDouble());
      recorder.addReading("Move Arm Left", -Robot.m_oi.ojoy.getRawAxis(5));
      recorder.addReading("Move Arm Right", Robot.m_oi.ojoy.getRawAxis(5));
      recorder.addReading("Arm is extended", ArmExtension.convertBoolToDouble());
      recorder.addReading("Arm is retracted", ArmExtension.convertBoolToDouble());
			System.out.println(recorder.initNextReading());
		}
		
		else if (Recorder.isStoring()) {
			recorder.storeWritings();
		}*/
   
    Scheduler.getInstance().run();
  }
  public void talonInitVelocity(TalonSRX talon) {
    /* Factory Default all hardware to prevent unexpected behaviour */
    talon.configFactoryDefault();

   /* Config sensor used for Primary PID [Velocity] */
    talon.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder,
    Constants.kPIDLoopIdx, 
    Constants.kTimeoutMs);

    

   /* Config the peak and nominal outputs */
   talon.configNominalOutputForward(0, Constants.kTimeoutMs);
   talon.configNominalOutputReverse(0, Constants.kTimeoutMs);
   talon.configPeakOutputForward(1, Constants.kTimeoutMs);
   talon.configPeakOutputReverse(-1, Constants.kTimeoutMs);

   /* Config the Velocity closed loop gains in slot0 */
   talon.config_kF(Constants.kPIDLoopIdx, Constants.kGains_Velocit.kF, Constants.kTimeoutMs);
   talon.config_kP(Constants.kPIDLoopIdx, Constants.kGains_Velocit.kP, Constants.kTimeoutMs);
   talon.config_kI(Constants.kPIDLoopIdx, Constants.kGains_Velocit.kI, Constants.kTimeoutMs);
   talon.config_kD(Constants.kPIDLoopIdx, Constants.kGains_Velocit.kD, Constants.kTimeoutMs);
}


//MIGHT NEED TO DELETE

public void talonInitPosition(TalonSRX talon)
{
  /* Config the sensor used for Primary PID [POSITION] and sensor direction */
  talon.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 
  Constants.kPIDLoopIdx,
Constants.kTimeoutMs);

/* Ensure sensor is positive when output is positive */
talon.setSensorPhase(Constants.kSensorPhase);

/**
* Set based on what direction you want forward/positive to be.
* This does not affect sensor phase. 
*/ 
talon.setInverted(Constants.kMotorInvert);

/* Config the peak and nominal outputs, 12V means full */
talon.configNominalOutputForward(0, Constants.kTimeoutMs);
talon.configNominalOutputReverse(0, Constants.kTimeoutMs);
talon.configPeakOutputForward(1, Constants.kTimeoutMs);
talon.configPeakOutputReverse(-1, Constants.kTimeoutMs);

/**
* Config the allowable closed-loop error, Closed-Loop output will be
* neutral within this range. See Table in Section 17.2.1 for native
* units per rotation.
*/
talon.configAllowableClosedloopError(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);

/* Config Position Closed Loop gains in slot0, tsypically kF stays zero. */
talon.config_kF(Constants.kPIDLoopIdx, Constants.kGains.kF, Constants.kTimeoutMs);
talon.config_kP(Constants.kPIDLoopIdx, Constants.kGains.kP, Constants.kTimeoutMs);
talon.config_kI(Constants.kPIDLoopIdx, Constants.kGains.kI, Constants.kTimeoutMs);
talon.config_kD(Constants.kPIDLoopIdx, Constants.kGains.kD, Constants.kTimeoutMs);

/**
* Grab the 360 degree position of the MagEncoder's absolute
* position, and intitally set the relative sensor to match.
*/
int absolutePosition = talon.getSensorCollection().getPulseWidthPosition();

/* Mask out overflows, keep bottom 12 bits */
absolutePosition &= 0xFFF;
if (Constants.kSensorPhase) { absolutePosition *= -1; }
if (Constants.kMotorInvert) { absolutePosition *= -1; }

/* Set the quadrature (relative) sensor to match absolute */
talon.setSelectedSensorPosition(absolutePosition, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
}

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
    Robot.arm.armMotorR.set(ControlMode.PercentOutput, Robot.m_oi.ojoy.getRawAxis(1));
    Robot.arm.armMotorL.set(ControlMode.PercentOutput, -1 * Robot.m_oi.ojoy.getRawAxis(1));
    
  }
  public void init(){
    recorder.addFileSelect(numpaths, path);
		SmartDashboard.putString(Integer.toString(numpaths), path);
		++numpaths;
		path = new String("/home/lvuser/RightSwitchFromLeft.txt");
		recorder.addFileSelect(numpaths, path);
		SmartDashboard.putString(Integer.toString(numpaths), path);
		++numpaths;
  }
}
