package org.usfirst.frc4729.FRC2019.commands;

import org.usfirst.frc4729.FRC2019.Robot;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.Subsystem;

public class Auto extends CommandGroup {
  
  public Auto() {
    addParallel(new SetWinchLevel(0)); //start at level 0

    addSequential(new FollowGaffer());
  }
}
