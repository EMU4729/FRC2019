/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                                                         */
/* Open Source Software - may be modified and shared by FRC teams. The code     */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                                                                                             */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc4729.FRC2019.commands;

import org.usfirst.frc4729.FRC2019.subsystems.Navigation;
import edu.wpi.first.wpilibj.command.CommandGroup;

public class Test extends CommandGroup {
    public Test() {
        addSequential(new SetLastKnownLocation(Navigation.LEVEL_1_CENTER));
        addSequential(new FollowRetroreflective(Navigation.CARGO_END_LEFT));
    }
}
