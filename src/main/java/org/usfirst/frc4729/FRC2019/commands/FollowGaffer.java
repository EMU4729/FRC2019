package org.usfirst.frc4729.FRC2019.commands;

import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc4729.FRC2019.Robot;
import org.usfirst.frc4729.FRC2019.Util;

public class FollowGaffer extends Command {
    public FollowGaffer() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
        requires(Robot.drive);
        requires(Robot.vision);
        requires(Robot.distanceSensor);
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
    }

    double offsetSlowRange = 50;
	double offsetStopRange = 0;
    double offsetMinPower = 0.5;
    double offsetMaxPower = 1;

    double angleSlowRange = 50;
    double angleStopRange = 0;
    double angleMinPower = 0.5;
    double angleMaxPower = 1;

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
        double forwards = 0;
        if (Robot.vision.gafferEndVisible) {
            forwards = 1;
        }
        Robot.drive.omni(forwards,
                         Util.linear(Robot.vision.gafferOffsetX,
                                     0,
                                     offsetMinPower,
                                     offsetMaxPower,
                                     offsetSlowRange,
                                     offsetStopRange),
                         Util.linear(Robot.vision.gafferAngle,
                                     0,
                                     angleMinPower,
                                     angleMaxPower,
                                     angleSlowRange,
                                     angleStopRange));
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished() {
        if (Robot.distanceSensor.isTouchingWall()) {
            Robot.gyro.reset();
            return true;
        } else {
            return false;
        }
    }

    // Called once after isFinished returns true
    @Override
    protected void end() {
        // Robot.drive.omni(0, 0, 0);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted() {
    }
}
