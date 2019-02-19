package org.usfirst.frc4729.FRC2019.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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
    double offsetMinPower = 0;
    double offsetMaxPower = 1;

    double angleSlowRange = 50;
    double angleStopRange = 0;
    double angleMinPower = 0;
    double angleMaxPower = 1;

    double offsetGoRange = 10;
    double angleGoRange = 10;

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
        double forwards = 0;
        if (Robot.vision.gafferEndVisible
            || (Util.isInRange(Robot.vision.gafferOffsetX, 0, offsetGoRange)
                && Util.isInRange(Robot.vision.gafferAngle, 0, angleGoRange))) {
            forwards = 1;
        }
        
        double sideways = Util.linear(Robot.vision.gafferOffsetX,
                                      0,
                                      offsetMinPower,
                                      offsetMaxPower,
                                      offsetSlowRange,
                                      offsetStopRange);

        double turn = Util.linear(Robot.vision.gafferAngle,
                                  0,
                                  angleMinPower,
                                  angleMaxPower,
                                  angleSlowRange,
                                  angleStopRange);

        // SmartDashboard.putNumber("forwards", forwards);
        // SmartDashboard.putNumber("sideways", sideways);
        // SmartDashboard.putNumber("turn", turn);

        Robot.drive.omni(forwards, sideways, turn);
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished() {
        return Robot.distanceSensor.isTouchingWall();
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
