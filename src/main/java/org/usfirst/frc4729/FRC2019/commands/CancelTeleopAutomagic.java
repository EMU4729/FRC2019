/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                                                         */
/* Open Source Software - may be modified and shared by FRC teams. The code     */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                                                                                             */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc4729.FRC2019.commands;

import edu.wpi.first.wpilibj.command.InstantCommand;
import org.usfirst.frc4729.FRC2019.Robot;

/**
 * Add your docs here.
 */
public class CancelTeleopAutomagic extends InstantCommand {
    /**
     * Add your docs here.
     */
    public CancelTeleopAutomagic() {
        super();
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
        requires(Robot.drive);
        requires(Robot.mechanism);
    }

    // Called once when the command executes
    @Override
    protected void initialize() {
    }
}
