/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                                                         */
/* Open Source Software - may be modified and shared by FRC teams. The code     */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                                                                                             */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc4729.FRC2019.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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
    double edgeRange = 0.5;

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
        double forwards = 1;

        double angle = Robot.vision.retroreflectiveRelativeAngle;
        if (angle < -45) angle = -45;
        if (angle >  45) angle =  45;
        // SmartDashboard.putNumber("sideways sin", Math.sin(Math.toRadians(angle)));
        // SmartDashboard.putNumber("sideways cos", Math.cos(Math.toRadians(angle)));
        // SmartDashboard.putNumber("retroreflective angle", Robot.vision.retroreflectiveRelativeAngle);
        // SmartDashboard.putNumber("clipped angle", angle);
        double sideways =   Math.sin(Math.toRadians(angle))
                          / Math.cos(Math.toRadians(angle));

        double turn = Util.linear(Robot.navigation.relativeAngle(location.angle),
                                  0,
                                  minPower,
                                  maxPower,
                                  slowRange,
                                  stopRange);
                                  
        // if (Robot.vision.retroreflectiveAvailable) {
            double distanceToEdge = (1 - Math.abs(Robot.vision.retroreflectiveRelativeAngle) / Robot.vision.cameraConeHalfAngle);
            if (distanceToEdge < edgeRange) {
                turn = Math.signum(Robot.vision.retroreflectiveRelativeAngle);
                forwards = 0;
            }
        // }

        SmartDashboard.putNumber("forwards", forwards);
        SmartDashboard.putNumber("sideways", sideways);
        SmartDashboard.putNumber("turn", turn);

        Robot.drive.omni(forwards, sideways, turn);
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished() {
        return false;// Robot.vision.gafferAvailable; // TODO
    }

    // Called once after isFinished returns true
    @Override
    protected void end() {
        Robot.drive.omni(0, 0, 0);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted() {
        end();
    }
}
