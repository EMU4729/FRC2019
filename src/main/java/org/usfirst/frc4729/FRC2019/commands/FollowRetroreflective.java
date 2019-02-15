/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                                                         */
/* Open Source Software - may be modified and shared by FRC teams. The code     */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                                                                                             */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc4729.FRC2019.commands;

import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc4729.FRC2019.Robot;
import org.usfirst.frc4729.FRC2019.Util;
import org.usfirst.frc4729.FRC2019.subsystems.Navigation.Location;

public class FollowRetroreflective extends Command {
    Location location;

    public FollowRetroreflective(Location location) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
        requires(Robot.drive);
        requires(Robot.vision);
        this.location = location;
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
    }

    double slowRange = 20;
    double stopRange = 0;
    double minPower = 0;
    double maxPower = 1;

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
        double forwards = 1;
        double sideways = sidewaysFromAngle(Robot.vision.retroreflectiveRelativeAngle);
        double turn = Util.linear(Robot.navigation.relativeAngle(location.angle),
                                  0,
                                  minPower,
                                  maxPower,
                                  slowRange,
                                  stopRange)
                      * (1 - Math.abs(Robot.vision.retroreflectiveRelativeAngle) / Robot.vision.cameraConeHalfAngle);

        Robot.drive.omni(forwards, sideways, turn);
    }

    private double sidewaysFromAngle(double angle) {
        double y = Math.sin(angle);
        double x = Math.cos(angle);
        double sideways = (1 / y) * x;
        if (sideways > 1) sideways = 1;
        return sideways;
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished() {
        return Robot.vision.gafferAvailable;
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
