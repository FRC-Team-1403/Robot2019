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
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.SwitchMode;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.DriveTrain;
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

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  public static ExampleSubsystem m_subsystem = new ExampleSubsystem();
  public static OI m_oi;
  public static DriveTrain drivetrain;
  public static Intake in;
  public static ArmExtension p;
  public static Arm arm;
  public static Wrist w;
  public static ControlSystem cs;

  public static double FASTMAXRPM = 500;
  public static double DEFAULTMAXRPM = 600;
  public static double FINECONTROLRPM = 100;
  public static double maxRPM = DEFAULTMAXRPM;

  public static double FINECONTROLMAXACCELERATION = 100;
  public static double DEFAULTMAXACCELERATION = 0.0002;
  public static double HYPERSPEEDMAXACCELERATION = 10000;

  public static NetworkTable table;
  public static NetworkTableEntry tx;
  public static NetworkTableEntry tv;
  public static NetworkTableEntry ta;
  Command m_autonomousCommand;
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    m_oi = new OI();
    drivetrain = new DriveTrain();
    in = new Intake();
    p =  new ArmExtension();
    arm = new Arm();
    w = new Wrist();
    cs = new ControlSystem();
    m_chooser.setDefaultOption("Default Auto", new ExampleCommand());
    // chooser.addOption("My Auto", new MyAutoCommand());
    SmartDashboard.putData("Auto mode", m_chooser);
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
    if (m_autonomousCommand != null) {
      m_autonomousCommand.start();
    }
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    Scheduler.getInstance().run();
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

    SmartDashboard.putNumber("Motor value 1: ", drivetrain.frontLeft.getMotorOutputPercent());
    SmartDashboard.putNumber("Motor value 2: ", drivetrain.backLeft.getMotorOutputPercent());
    SmartDashboard.putNumber("Motor value 3: ", drivetrain.frontRight.getMotorOutputPercent());
    SmartDashboard.putNumber("Motor value 4: ", drivetrain.backRight.getMotorOutputPercent());

    table = NetworkTableInstance.getDefault().getTable("limelight");
    tx = table.getEntry("tx");
    tv = table.getEntry("tv");
    ta = table.getEntry("ta");
    
   double x = tx.getDouble(0.0);
   double v = tv.getDouble(2.0);

    if (m_oi.ojoy.getRawButton(3)) {
      if ((int)(v) == 1) {
        
        Vision.isInMiddle(x);
        Vision.align(x);
      }  else {
        
        System.out.println("No target!!!!!!!");
      } 
    }
    if (m_oi.ojoy.getRawButtonReleased(3)) {
      System.out.println("This is when the manipulator is supposed to happen!!!");
      Vision.reset();
    }

    if (m_oi.ojoy.getRawButton(5)) {
     if ((int)(v) == 1) {
       Vision.isInMiddle(tx.getDouble(0.0));
     }
     Vision.alignLeft();
   }
   if (m_oi.ojoy.getRawButtonReleased(5)) {
     System.out.println("This is when the manipulator is supposed to happen!!!");
     Vision.reset();
   }

   if (m_oi.ojoy.getRawButton(6)) {
     if ((int)(v) == 1) {
       Vision.isInMiddle(tx.getDouble(0.0));
     }
     Vision.alignRight();
   }
   if (m_oi.ojoy.getRawButtonReleased(6)) {
     System.out.println("This is when the manipulator is supposed to happen!!!");
     Vision.reset();
   }
   if(m_oi.ojoy.getRawButton(2)){
     drivetrain.moveBackward();
   }
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
  }
}
