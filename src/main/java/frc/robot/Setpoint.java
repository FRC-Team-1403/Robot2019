package frc.robot;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import frc.robot.Robot;

public class Setpoint {
    double armAngle;
    double wristAngle;
    public boolean armOut;


    public Setpoint(double armPotReading, double wristPotReading) {
        this.armAngle = armPotReading;
        this.wristAngle = wristPotReading;
        armOut = false;
    }
    
    public Setpoint(double armPotReading, double wristPotReading, boolean armOut){
        this.armAngle = armPotReading;
        this.wristAngle = wristPotReading;
        this.armOut = armOut;
    }

    public void run(){
        Robot.arm.setpoint = armAngle;
        Robot.w.setpoint =  wristAngle;
        if(armOut){
            Robot.in.hatchPush.set(DoubleSolenoid.Value.kForward);
        } 

        else if(!armOut) {
            if(Robot.in.hatchPush.get() == DoubleSolenoid.Value.kForward) {
                Robot.in.hatchPush.set(DoubleSolenoid.Value.kReverse);
            }
        }

    }
    
}