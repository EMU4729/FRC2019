/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc4729.FRC2019;

/**
 * Add your docs here.
 */
public class Navigation {
    // public

    public static double angle(Point from, Point to) {
        return Math.toDegrees(Math.atan2(to.y - from.y, to.x - from.x));
    }

    public static class Point {
        double x = 0;
        double y = 0;

        public Point(double x, double y) {
            this.x = x;
            this.y = y;
        }
    }
}