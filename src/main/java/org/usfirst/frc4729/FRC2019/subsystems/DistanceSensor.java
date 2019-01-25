/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc4729.FRC2019.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.Ultrasonic;

/**
 * Add your docs here.
 */
public class DistanceSensor extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  Ultrasonic ultrasonic;

  public static final double touchingWallDistance = 20;

  public DistanceSensor() {
    ultrasonic = new Ultrasonic(1, 2); // TODO dab
  }

  public double getDistance() {
    return ultrasonic.getRangeMM();
  }

  public boolean isTouchingWall() {
    return (getDistance() <= touchingWallDistance);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

}
