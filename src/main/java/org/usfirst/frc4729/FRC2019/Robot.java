// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


package org.usfirst.frc4729.FRC2019;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Joystick;
import org.usfirst.frc4729.FRC2019.commands.*;
import org.usfirst.frc4729.FRC2019.subsystems.*;
import org.usfirst.frc4729.FRC2019.subsystems.Navigation.Location;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.properties file in 
 * the project.
 */
public class Robot extends TimedRobot {
    Command auto;

    SendableChooser<Boolean> autoDo = new SendableChooser<>();
    SendableChooser<Location> autoStart = new SendableChooser<>();
    SendableChooser<Location> autoFirstCargoEnd = new SendableChooser<>();
    SendableChooser<Location> autoFirstLoadingStation = new SendableChooser<>();

    public static OI oi;
    public static Drive drive;
    public static Mechanism mechanism;
    public static Vision vision;
    public static Navigation navigation;
    public static DistanceSensor distanceSensor;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS

    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    @Override
    public void robotInit() {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
        drive = new Drive();
        mechanism = new Mechanism();
        vision = new Vision();
        vision.startCameras();
        navigation = new Navigation();
        distanceSensor = new DistanceSensor();

        // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
        // OI must be constructed after subsystems. If the OI creates Commands
        //(which it very likely will), subsystems are not guaranteed to be
        // constructed yet. Thus, their requires() statements may grab null
        // pointers. Bad news. Don't move it.
        oi = new OI();

        // Add commands to Autonomous Sendable Chooser
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=AUTONOMOUS

        autoDo.setDefaultOption("Do nothing", Boolean.FALSE);
        autoDo.addOption("Run autonomous", Boolean.TRUE);

        autoStart.addOption("Level 1, left", Navigation.LEVEL_1_LEFT);
        autoStart.setDefaultOption("Level 1, centre", Navigation.LEVEL_1_CENTER);
        autoStart.addOption("Level 1, right", Navigation.LEVEL_1_LEFT);
        autoStart.addOption("Level 2, left", Navigation.LEVEL_2_LEFT);
        autoStart.addOption("Level 2, right", Navigation.LEVEL_2_RIGHT);

        autoFirstCargoEnd.setDefaultOption("Automatic", null);
        autoFirstCargoEnd.addOption("Left hatch first", Navigation.CARGO_END_LEFT);
        autoFirstCargoEnd.addOption("Right hatch first", Navigation.CARGO_END_RIGHT);

        autoFirstLoadingStation.setDefaultOption("Automatic", null);
        autoFirstLoadingStation.addOption("Left loading station first", Navigation.LOADING_STATION_LEFT);
        autoFirstLoadingStation.addOption("Right loading station first", Navigation.LOADING_STATION_RIGHT);

        SmartDashboard.putData("Run autonomous", autoDo);
        SmartDashboard.putData("Start position", autoStart);
        SmartDashboard.putData("First hatch", autoFirstCargoEnd);
        SmartDashboard.putData("First loading station", autoFirstLoadingStation);
    }

    /**
     * This function is called when the disabled button is hit.
     * You can use it to reset subsystems before shutting down.
     */
    @Override
    public void disabledInit() {
    }

    @Override
    public void disabledPeriodic() {
        Scheduler.getInstance().run();
        // SmartDashboard.putNumber("ultrasonic mm", distanceSensor.getDistance());
    }

    @Override
    public void autonomousInit() {
        if (autoDo.getSelected().booleanValue()) {
            // auto = new Auto(autoStart.getSelected(), autoFirstCargoEnd.getSelected(), autoFirstLoadingStation.getSelected());
            auto = new Test();
            if (auto != null) auto.start();
        }
    }

    @Override
    public void autonomousPeriodic() {
        Scheduler.getInstance().run();
    }

    @Override
    public void teleopInit() {
        if (auto != null) auto.cancel();
        if (!Robot.mechanism.winchCalibrated) {
            Command calibrateWinch = new CalibrateWinch();
            calibrateWinch.start();
        }
        Omni omni = new Omni();
        omni.start();
    }

    @Override
    public void teleopPeriodic() {
        Scheduler.getInstance().run();
    }
}
