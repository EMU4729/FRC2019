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
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
    }

    double offsetSlowRange = 50;
	double offsetStopRange = 10;
	
	double angleSlowRange = 50;
    double angleStopRange = 10;

    double minPower = 0.5;
    double maxPower = 1;

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
            Robot.drive.omni(1,
							 Util.linear(Robot.vision.gafferOffsetX,
										 0,
										 minPower,
										 maxPower,
										 offsetSlowRange,
										 offsetStopRange
										),
							 Util.linear(Robot.vision.gafferAngle,
										 0,
										 minPower,
										 maxPower,
										 angleSlowRange,
										 angleStopRange
										)
							);
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished() {
        return Robot.distanceSensor.isTouchingWall();
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
