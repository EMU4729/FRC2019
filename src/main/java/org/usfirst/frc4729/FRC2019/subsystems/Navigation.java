/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                                                         */
/* Open Source Software - may be modified and shared by FRC teams. The code     */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                                                                                             */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc4729.FRC2019.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import org.usfirst.frc4729.FRC2019.Robot;

/**
 * Add your docs here.
 */
public class Navigation extends Subsystem {
    // All navigation locations are defined by the blue side of the field
    // in the field diagram on PDF page 15 (paper page 11) of the manual.
    // The origin is the bottom left corner and the units of dimensions
    // are meters. The angle 0 degrees is to the right, angles rotate
    // counterclockwise and the units of angles are degrees.

    public static final Location CARGO_END_TOP = new Location(0, 0, 0);
    public static final Location CARGO_END_BOTTOM = new Location(0, 0, 0);
    public static final Location STATION_TOP = new Location(0, 0, 0);
    public static final Location STATION_BOTTOM = new Location(0, 0, 0);

    double referenceAngle = 0;

    public double getRobotAngle() {
        return referenceAngle + Robot.gyro.getAngle();
    }

    public static double angleBetweenLocations(Location from, Location to) {
        return Math.toDegrees(Math.atan2(to.y - from.y, to.x - from.x));
    }

    public static class Location {
        double x = 0;
        double y = 0;
        double angle = 0;

        public Location(double x, double y, double angle) {
            this.x = x;
            this.y = y;
            this.angle = angle;
        }
    }

    @Override
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        // setDefaultCommand(new MySpecialCommand());
    }
}
