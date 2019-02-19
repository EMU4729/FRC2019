/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                                                         */
/* Open Source Software - may be modified and shared by FRC teams. The code     */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                                                                                             */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc4729.FRC2019.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import org.usfirst.frc4729.FRC2019.Util;
import edu.wpi.first.wpilibj.AnalogGyro;

/**
 * Add your docs here.
 */
public class Navigation extends Subsystem {
    // All navigation locations are defined by the blue side of the field
    // in the field diagram on PDF page 15 (paper page 11) of the manual, but
    // rotated 90 degrees so the blue side is at the bottom, and the red side is
    // at the top. The origin is the bottom left corner and the units of
    // dimensions are millimeters. The angle 0 degrees is to the right, angles
    // rotate counterclockwise and the units of angles are degrees.

    public static final Location LEVEL_1_LEFT = new Location(3029, 1667, 90);
    public static final Location LEVEL_1_CENTER = new Location(4091, 1667, 90);
    public static final Location LEVEL_1_RIGHT = new Location(5163, 1667, 90);
    public static final Location LEVEL_2_LEFT = new Location(2956, 626, 90);
    public static final Location LEVEL_2_RIGHT = new Location(5236, 626, 90);

    public static final Location CARGO_END_LEFT = new Location(3820, 5671, 90);
    public static final Location CARGO_END_RIGHT = new Location(4372, 5671, 90);
    public static final Location LOADING_STATION_LEFT = new Location(660, 0, 270);
    public static final Location LOADING_STATION_RIGHT = new Location(7532, 0, 270);

    // public AnalogGyro gyro; // TODO
    public Location lastKnownLocation = null;
    public double referenceAngle = 0;

    public Navigation() {
        // gyro = new AnalogGyro(1);
    }

    public double getRobotAngle() {
        return 0;//Util.normAngle(referenceAngle + gyro.getAngle());
    }

    public double relativeAngle(double angle) {
        return Util.normAngle(getRobotAngle() - angle) - 180;
    }

    public double angleToLocation(Location location) {
        return angleBetweenLocations(lastKnownLocation, location);
    }

    public void setLastKnownLocation(Location location) {
        lastKnownLocation = location;
        // gyro.reset();
    }

    public static double angleBetweenLocations(Location from, Location to) {
        return Math.toDegrees(Math.atan2(to.y - from.y, to.x - from.x));
    }

    public static class Location {
        public double x = 0;
        public double y = 0;
        public double angle = 0;

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
