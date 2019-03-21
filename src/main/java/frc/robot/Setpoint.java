package frc.robot;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import frc.robot.Robot;

public class Setpoint {
    double armPotReading;
    double wristPotReading;
    public boolean armOut;


    public Setpoint(double armPotReading, double wristPotReading) {
        this.armPotReading = armPotReading;
        this.wristPotReading = wristPotReading;
        armOut = false;
    }
    
    public Setpoint(double armPotReading, double wristPotReading, boolean armOut){
        this.armPotReading = armPotReading;
        this.wristPotReading = wristPotReading;
        this.armOut = armOut;
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
    }

}