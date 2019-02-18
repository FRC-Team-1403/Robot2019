package frc.robot;

public class Setpoint {
    double armPotReading;
    double wristPotReading;

    public Setpoint(double armPotReading, double wristPotReading) {
        this.armPotReading = armPotReading;
        this.wristPotReading = wristPotReading;
    }

    public double getArmAngle() {
        return Robot.arm.voltToRadians(armPotReading);
    }
    public double getWristAngle(){
        return Robot.w.voltToRadians(wristPotReading);
    }
}