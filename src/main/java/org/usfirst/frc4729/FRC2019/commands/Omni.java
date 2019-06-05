/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                                                         */
/* Open Source Software - may be modified and shared by FRC teams. The code     */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                                                                                             */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc4729.FRC2019.commands;

import org.usfirst.frc4729.FRC2019.Robot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Omni extends Command {
    // Joystick leftStick = new Joystick(0);
    // Joystick rightStick = new Joystick(1);

    private XboxController controller = new XboxController(0);

    public Omni() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
        requires(Robot.drive);
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
        // SmartDashboard.putNumber("forwards before processing ...", -controller.getY(Hand.kLeft));
        // SmartDashboard.putNumber("sideways before processing ...", controller.getX(Hand.kLeft));
        // SmartDashboard.putNumber("turn before processing ...", controller.getX(Hand.kRight));
        double turnFactor = 0.7;
        Robot.drive.omni(-controller.getY(Hand.kRight), controller.getX(Hand.kRight), controller.getX(Hand.kLeft) * turnFactor);
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
