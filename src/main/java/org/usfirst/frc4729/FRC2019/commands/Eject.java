/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                                                         */
/* Open Source Software - may be modified and shared by FRC teams. The code     */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                                                                                             */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc4729.FRC2019.commands;

import org.usfirst.frc4729.FRC2019.Robot;
import edu.wpi.first.wpilibj.command.TimedCommand;

public class Eject extends TimedCommand {
    private static final double EJECT_TIME = 1000; // milliseconds

    public Eject() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
        super(EJECT_TIME);
        requires(Robot.mechanism);
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
        Robot.mechanism.eject();
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
    }

    // Called once after isFinished returns true
    @Override
    protected void end() {
        Robot.mechanism.uneject();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted() {
        end();
    }
}
