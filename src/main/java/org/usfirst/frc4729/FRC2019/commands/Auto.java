/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                                                         */
/* Open Source Software - may be modified and shared by FRC teams. The code     */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                                                                                             */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc4729.FRC2019.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import org.usfirst.frc4729.FRC2019.subsystems.Navigation;
import org.usfirst.frc4729.FRC2019.subsystems.Navigation.Location;

public class Auto extends CommandGroup {
    /**
     * Add your docs here.
     */
    public Auto(Location start, Location firstCargoEnd, Location firstLoadingStation) {
        // Add Commands here:
        // e.g. addSequential(new Command1());
        // addSequential(new Command2());
        // these will run in order.

        // To run multiple commands at the same time,
        // use addParallel()
        // e.g. addParallel(new Command1());
        // addSequential(new Command2());
        // Command1 and Command2 will run in parallel.

        // A command group will require all of the subsystems that each member
        // would require.
        // e.g. if Command1 requires chassis, and Command2 requires arm,
        // a CommandGroup containing them would require both the chassis and the
        // arm.

        firstCargoEnd = first(firstCargoEnd, start, Navigation.CARGO_END_LEFT, Navigation.CARGO_END_RIGHT);
        firstLoadingStation = first(firstLoadingStation, start, Navigation.LOADING_STATION_LEFT, Navigation.LOADING_STATION_RIGHT);
        Location secondCargoEnd = second(firstCargoEnd, Navigation.CARGO_END_LEFT, Navigation.CARGO_END_RIGHT);
        Location secondLoadingStation = second(firstLoadingStation, Navigation.LOADING_STATION_LEFT, Navigation.LOADING_STATION_RIGHT);

        addSequential(new CalibrateWinch());
        addSequential(new SetLastKnownLocation(start));
        go(firstCargoEnd);
        go(firstLoadingStation);
        go(secondCargoEnd);
        go(secondLoadingStation);
    }

    private void go(Location location) {
        addSequential(new RotateTowards(location));
        addSequential(new FollowRetroreflective(location));
        addSequential(new FollowGaffer());
        addSequential(new SetWinchLevel(0)); 
        addSequential(new Eject());
        addSequential(new Retreat());
        addSequential(new SetLastKnownLocation(location));
    }

    private Location first(Location first, Location start, Location left, Location right) {
        if (first == null) {
            if (start == Navigation.LEVEL_1_LEFT || start == Navigation.LEVEL_2_LEFT || start == Navigation.LEVEL_1_CENTER) {
                first = left;
            } else {
                first = right;
            }
        }
        return first;
    }

    private Location second(Location first, Location left, Location right) {
        Location result = left;
        if (first == left) {
            result = right;
        }
        return result;
    }
}
