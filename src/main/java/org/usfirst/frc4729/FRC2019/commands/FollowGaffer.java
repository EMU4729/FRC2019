package org.usfirst.frc4729.FRC2019.commands;

import org.usfirst.frc4729.FRC2019.Robot;
import org.usfirst.frc4729.FRC2019.Util;
import org.usfirst.frc4729.FRC2019.subsystems.Vision;

import edu.wpi.first.wpilibj.command.Command;

public class FollowGaffer extends Command {
    Vision vision;

    public FollowGaffer() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
        requires(Robot.drive);
        requires(Robot.vision);
        vision = Robot.vision;
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
    }

    double slowRangeOffset = 50;
	double stopRangeOffset = 10;
	
	double slowRangeAngle = 50;
    double stopRangeAngle = 10;

    double minPower = 0.5;
    double maxPower = 1;

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
            Robot.drive.omni(1,
							 Util.linear(vision.gafferOffset.x,
										 vision.gafferCenter.x,
										 minPower,
										 maxPower,
										 slowRangeOffset,
										 stopRangeOffset
										),
							 Util.linear(vision.gafferAngle,
										 0,
										 minPower,
										 maxPower,
										 slowRangeAngle,
										 stopRangeAngle
										)
							);
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished() {
        Robot.drive.omni(0, 0, 0);
        return Robot.distanceSensor.isTouchingWall();
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
