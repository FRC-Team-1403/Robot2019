package frc.robot;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import frc.robot.Robot;

public class Setpoint {
    double armPotReading;
    double wristPotReading;
    public boolean armOut;
    public boolean servoOut;

    public Setpoint(double armPotReading, double wristPotReading) {
        this.armPotReading = armPotReading;
        this.wristPotReading = wristPotReading;
        armOut = false;
        servoOut = false;
    }
    
    public Setpoint(double armPotReading, double wristPotReading, boolean armOut, boolean servoOut){
        this.armPotReading = armPotReading;
        this.wristPotReading = wristPotReading;
        this.armOut = armOut;
        this.servoOut = servoOut;
    }

    public void run(){
        Robot.arm.setpoint = armPotReading;
        Robot.w.setpoint = wristPotReading;
        if(armOut && !Robot.ae.isForward){
            Robot.ae.armExtender.set(DoubleSolenoid.Value.kForward);
            Robot.ae.isForward = true;
         } 

        else if(!armOut) {
            Robot.ae.armExtender.set(DoubleSolenoid.Value.kReverse);
            Robot.ae.isForward = false;
        } 
        if(servoOut){
            Robot.hatch.hookServo.setPosition(1);
        }
        else{
        }
    }

}