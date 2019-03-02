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
        Robot.arm.setpoint = getArmAngle();
        Robot.w.setpoint =  getWristAngle();
        if(armOut){
            if(Robot.ae.armExtender.get() != DoubleSolenoid.Value.kForward) {
                Robot.ae.armExtender.set(DoubleSolenoid.Value.kForward);
            }
         } 

        else if(!armOut) {
            if(Robot.ae.armExtender.get() == DoubleSolenoid.Value.kForward) {
                Robot.ae.armExtender.set(DoubleSolenoid.Value.kReverse);
            }
        }

    }
    
    public double getArmAngle(){
        return Robot.arm.voltToRadians(armPotReading);
    }

    public double getWristAngle(){
        return Robot.w.voltToRadians(wristPotReading);
    }

}