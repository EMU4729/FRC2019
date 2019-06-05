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
    private double previous_time = 0;

    public PidMotor(int motorNum, double kp, double ki, double kd) {
        motor = new TalonSRX(motorNum);
        motor.setInverted(false);
        timer = new Timer();
        kp = this.kp;
        ki = this.ki;
        kd = this.kd;
    }

    public void Update(double setPoint, double turnError){
        double error = setPoint - previousError;
        
        integral += error * (timer.get() - previous_time);
        double derivative = (error - previousError) / (timer.get() - previous_time);
        double output = kp * error + ki * integral + kd * derivative;
        previousError = error;
        motor.set(ControlMode.PercentOutput, output);
    }
}
