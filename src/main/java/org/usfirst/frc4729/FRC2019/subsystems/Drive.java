package org.usfirst.frc4729.FRC2019.subsystems;

import java.util.List;
import java.util.Arrays;
import java.util.stream.Collectors;

import org.usfirst.frc4729.FRC2019.commands.*;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.SpeedController;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXPIDSetConfiguration;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;

public class Drive extends Subsystem {

    public double kp = 1; //poportionalised
    public double ki = 9; //interagal
    public double kd = 7; //differental
    public double kf = 3;
    public int izone = 5; //encoder ticks / analog units
    public double rampate = 6; //volts/second
    public int profile = 0; //0 or 1

    private TalonSRX leftFrontMotor;
    private TalonSRX rightFrontMotor;
    private TalonSRX leftBackMotor;
    private TalonSRX rightBackMotor;

    public Drive() {
        leftFrontMotor = new TalonSRX(0);
       // addChild("LeftFrontMotor",leftFrontMotor);
        leftFrontMotor.setInverted(false);
        
        rightFrontMotor = new TalonSRX(2);
       // addChild("RightFrontMotor",rightFrontMotor);
        rightFrontMotor.setInverted(false);
        
        leftBackMotor = new TalonSRX(4);
       // addChild("leftBackMotor",leftBackMotor);
        leftBackMotor.setInverted(false);
        
        rightBackMotor = new TalonSRX(5);
       // addChild("rightBackMotor",rightBackMotor);
        rightBackMotor.setInverted(false);
        
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
    
    public void omni(double forwards, double sideways, double turn) {
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

    public void setMotors(double leftFront, double leftBack, double rightFront, double rightBack) {
        leftFrontMotor.set(ControlMode.PercentOutput ,leftFront);
        leftBackMotor.set(ControlMode.PercentOutput ,leftBack);
        rightFrontMotor.set(ControlMode.PercentOutput ,rightFront);
        rightBackMotor.set(ControlMode.PercentOutput ,rightBack);
    }

    public void control () {
        TalonSRXPIDSetConfiguration talonconfig = new TalonSRXPIDSetConfiguration();
        talonconfig.selectedFeedbackCoefficient = 2.0;
        talonconfig.selectedFeedbackSensor = FeedbackDevice.QuadEncoder;
        leftFrontMotor.configurePID(talonconfig);
        leftBackMotor.configurePID(talonconfig);
        rightFrontMotor.configurePID(talonconfig);
        rightBackMotor.configurePID(talonconfig);
    }

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CMDPIDGETTERS


    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CMDPIDGETTERS

    // Put methods for controlling this subsystem
    // here. Call these from Commands.

}

