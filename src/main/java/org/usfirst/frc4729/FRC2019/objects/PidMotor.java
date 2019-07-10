/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc4729.FRC2019.objects;

import java.sql.Time;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Encoder;

/**
 * Add your docs here.
 */
public class PidMotor {

    public TalonSRX motor;
    public Timer timer;

    private Double kp;
    private Double ki;
    private Double kd;
    private double previousError = 0;
    private double integral = 0;
    private double previousTime = 0;
    private double currentSpeed = 0;
    private Encoder encoder;

    public PidMotor(int motorNum, double kp, double ki, double kd, int encoderPortA, int encoderPortB, boolean reverseEncoder) {
        motor = new TalonSRX(motorNum);
        motor.setInverted(false);
        timer = new Timer();
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
        encoder = new Encoder(encoderPortA, encoderPortB, false, Encoder.EncodingType.k4X);
    	encoder.setMaxPeriod(0.1);
        encoder.setMinRate(10);
        encoder.setDistancePerPulse(0.0762*Math.PI);
        encoder.setSamplesToAverage(7);
        encoder.setReverseDirection(false);
    }

    public void Update(double setPoint, double turnError) {
        if (!hasStarted){
            hasStarted = true;
            timer.start();
        }
        
        double error = setPoint - GetEncoderVelocity();// + turnError; 
        
        integral += error * (timer.get() - previousTime);
        double derivative = (error - previousError) / (timer.get() - previousTime);
        currentSpeed = kp * error + ki * integral + kd * derivative;
        previousError = error;
        motor.set(ControlMode.PercentOutput, currentSpeed/2);
    }

    public double GetCurrentSpeed() {
        return currentSpeed;
    }

    public double GetEncoderVelocity() {
        return encoder.getRate();
    }
    boolean hasStarted;
}
