/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                                                         */
/* Open Source Software - may be modified and shared by FRC teams. The code     */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                                                                                             */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc4729.FRC2019.commands;

import org.usfirst.frc4729.FRC2019.Robot;

import edu.wpi.first.wpilibj.command.InstantCommand;

public class ToggleSlowMode extends InstantCommand {
    public ToggleSlowMode() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
        super();
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
        Robot.drive.toggleSlowMode();
    }
}
