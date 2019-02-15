/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                                                         */
/* Open Source Software - may be modified and shared by FRC teams. The code     */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                                                                                             */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc4729.FRC2019.commands;

import org.usfirst.frc4729.FRC2019.Robot;
import edu.wpi.first.wpilibj.command.InstantCommand;
import org.usfirst.frc4729.FRC2019.subsystems.Navigation.Location;

public class SetLastKnownLocation extends InstantCommand {
    Location location;

    public SetLastKnownLocation(Location location) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
        super();
        requires(Robot.navigation);
        this.location = location;
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
        Robot.navigation.setLastKnownLocation(location);
    }
}