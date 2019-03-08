package org.usfirst.frc4729.FRC2019.subsystems;

import java.util.List;
import java.util.Arrays;
import java.util.stream.Collectors;

import org.usfirst.frc4729.FRC2019.commands.*;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.SpeedController;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXPIDSetConfiguration;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;

public class Drive extends Subsystem {

    // public double kp = 1; //poportionalised
    // public double ki = 9; //interagal
    // public double kd = 7; //differental
    // public double kf = 3;
    // public int izone = 5; //encoder ticks / analog units
    // public double rampate = 6; //volts/second
    // public int profile = 0; //0 or 1

    // private TalonSRX leftFrontMotor;
    // private TalonSRX rightFrontMotor;
    // private TalonSRX leftBackMotor;
    // private TalonSRX rightBackMotor;

    public Drive() {
        // leftFrontMotor = new TalonSRX(6);
        // leftFrontMotor.setInverted(false);
        
        // rightFrontMotor = new TalonSRX(11);
        // rightFrontMotor.setInverted(false);
        
        // leftBackMotor = new TalonSRX(5);
        // leftBackMotor.setInverted(false);
        
        // rightBackMotor = new TalonSRX(12);
        // rightBackMotor.setInverted(false);
    }

    @Override
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        // setDefaultCommand(new MySpecialCommand());
    }
    
    @Override
    public void periodic() {
        // Put code here to be run every loop

    }

    private static final double DEADZONE = 0.15;
    private static final double SPEED = 1;
    
    public void omni(double forwards, double sideways, double turn) {
        forwards = deadzoneMap(forwards) * SPEED;
        sideways = deadzoneMap(sideways) * SPEED;
        turn = deadzoneMap(turn) * SPEED;

        List<Double> power = Arrays.asList(forwards + sideways + turn,  // leftFront
                                           forwards - sideways + turn,  // leftBack
                                          -forwards + sideways + turn,  // rightFront
                                          -forwards - sideways + turn); // rightBack
                                          
        double max = power.stream().map(p -> Math.abs(p)).max((a, b) -> {
            Double difference = a - b;
            return difference.intValue();
        }).get();

        if (max > 1) {
            power = power.stream().map(p -> p / max).collect(Collectors.toList());
        }

        setMotors(power.get(0).doubleValue(),
                  power.get(1).doubleValue(),
                  power.get(2).doubleValue(),
                  power.get(3).doubleValue());
    }

    private double deadzoneMap(double value) {
        if (value < DEADZONE) {
            return 0;
        } else {
            return (value - DEADZONE) * (1 / (1 - DEADZONE));
        }
    }

    public void setMotors(double leftFront, double leftBack, double rightFront, double rightBack) {
        // leftFrontMotor.set(ControlMode.PercentOutput, leftFront);
        // leftBackMotor.set(ControlMode.PercentOutput, leftBack);
        // rightFrontMotor.set(ControlMode.PercentOutput, rightFront);
        // rightBackMotor.set(ControlMode.PercentOutput, rightBack);
    }

    public void control() {
        // TalonSRXPIDSetConfiguration config = new TalonSRXPIDSetConfiguration();
        // config.selectedFeedbackCoefficient = 2.0;
        // config.selectedFeedbackSensor = FeedbackDevice.QuadEncoder;
        // leftFrontMotor.configurePID(config);
        // leftBackMotor.configurePID(config);
        // rightFrontMotor.configurePID(config);
        // rightBackMotor.configurePID(config);
    }

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CMDPIDGETTERS


    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CMDPIDGETTERS

    // Put methods for controlling this subsystem
    // here. Call these from Commands.

}

