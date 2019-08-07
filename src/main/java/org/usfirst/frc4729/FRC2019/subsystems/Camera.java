/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc4729.FRC2019.subsystems;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoSource.ConnectionStrategy;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Add your docs here.
 */
public class Camera extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private static final int width = 320;
  private static final int height = 240;

  UsbCamera camera;
  CvSource outputStreams;
  CvSink sink;

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public Camera() {
      
      CameraServer instance = CameraServer.getInstance();

      camera = instance.startAutomaticCapture(0);
      camera.setResolution(width, height);
      camera.setConnectionStrategy(ConnectionStrategy.kKeepOpen);
      outputStreams = instance.putVideo("camera", width, height);
      // sink = instance.getVideo("USB Camera");
      // setupCamera(0, outputStreams, sink);
      
      // camera.setExposureManual(20);
  }
}
