package org.usfirst.frc4729.FRC2019.commands;

import org.usfirst.frc4729.FRC2019.Robot;

import edu.wpi.first.wpilibj.command.Command;

public class Align extends Command {
  public Align() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.vision);
    requires(Robot.drive);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    double forwards = 0;
    double sideways = 0;
    double turn = 0;

    //is the line in the center
    if (Robot.vision.offset.x > 10){
      //go right
      sideways = 1;
    } else if (Robot.vision.offset.y < -10) {
      //go left
      sideways = -1;
    }

    //can you see end of line
    if (Robot.vision.endVisible == true) {
      //go foward
      forwards = 1;
    } else if (Robot.vision.endVisible == false) {
      //stop
      forwards = 0;
    }

    //is the line straight
    if (Robot.vision.angle > 5) {
      //rotate right
      turn = 1;
    } else if (Robot.vision.angle < -5) {
      //rotate left
      turn = -1;
    }

    Robot.drive.omni(forwards, sideways, turn);
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
