package org.usfirst.frc4729.FRC2019.subsystems;

import java.util.List;
import java.awt.Robot;
import java.util.Arrays;
import java.util.stream.Collectors;

import org.usfirst.frc4729.FRC2019.commands.*;
import org.usfirst.frc4729.FRC2019.objects.PidMotor;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
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

    public static double MAX_SPEED = 2; //m/s
    public static double MAX_TURNING_SPEED = 0.016; //m/s 

    private PidMotor leftFrontMotor;
    private PidMotor rightFrontMotor;
    private PidMotor leftBackMotor;
    private PidMotor rightBackMotor;
    private ADXRS450_Gyro gyro;

    public Drive(ADXRS450_Gyro gyro) {
        leftFrontMotor = new PidMotor(6, 0, 0, 0);
        rightFrontMotor = new PidMotor(11, 0, 0, 0);      
        leftBackMotor = new PidMotor(5, 0, 0, 0);        
        rightBackMotor = new PidMotor(12, 0, 0, 0);

        gyro = this.gyro;
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

        double turnError = turn - gyro.getRate();

        setMotors(power.get(0).doubleValue(),
                  power.get(1).doubleValue(),
                  power.get(2).doubleValue(),
                  power.get(3).doubleValue(),
                  turnError);
    }

    private double deadzoneMap(double value) {
        if (value < DEADZONE) {
            return 0;
        } else {
            return (value - DEADZONE) * (1 / (1 - DEADZONE));
        }
    }

    public void setMotors(double leftFront, double leftBack, double rightFront, double rightBack, double turnError) {
        leftFrontMotor.Update(leftFront, turnError);
        leftBackMotor.Update(leftBack, turnError);
        rightFrontMotor.Update(rightFront, turnError);
        rightBackMotor.Update(rightBack, turnError);
    }
}

