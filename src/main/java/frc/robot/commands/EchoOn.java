package frc.robot.commands;

import frc.robot.echo.Recorder;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class EchoOn extends Command {
	
public EchoOn() {
		
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
		Recorder.startRecording();
		System.out.println("Recorder Recording Status: " + Recorder.isRecording);
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		
		return;
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		return true;
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