/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;

public class ReleasingTheHatchPanel extends CommandGroup {

  public ReleasingTheHatchPanel() {
   
    addSequential(new UnhookHatch());
    addSequential(new WaitCommand(0.5));
    addSequential(new PushHatch());
    addSequential(new ReleaseHatch());

  }
}
