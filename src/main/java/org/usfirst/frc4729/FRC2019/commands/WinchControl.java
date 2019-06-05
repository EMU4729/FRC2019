/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                                                         */
/* Open Source Software - may be modified and shared by FRC teams. The code     */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                                                                                             */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc4729.FRC2019.commands;

import org.usfirst.frc4729.FRC2019.Robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.command.Command;

public class WinchControl extends Command {
    private XboxController controller = new XboxController(0);

    public WinchControl() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
        //requires(Robot.mechanism);
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
        double up = controller.getYButton() ? 1 : 0;
        double down = controller.getBButton() ? 1 : 0;
        //Robot.mechanism.move(up - down);
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished() {
        return false;
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
