package org.usfirst.frc4729.FRC2019.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import org.usfirst.frc4729.FRC2019.Util;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.PWMTalonSRX;

public class Mechanism extends Subsystem {
    private int numPneumatics = 1;
    private DoubleSolenoid[] pneumatics;
    // private Compressor compressor;
    private TalonSRX winch;
    private int numCalls = 0;

    public Mechanism() {
        pneumatics = new DoubleSolenoid[numPneumatics];
        for (int i = 0; i < numPneumatics; i++) {
            pneumatics[i] = new DoubleSolenoid(i * 2 + 2, i * 2 + 1 + 2);
        }
        
        // compressor = new Compressor(0);

        winch = new TalonSRX(4);
        winch.setInverted(false);
    }

    @Override
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        // setDefaultCommand(new MySpecialCommand());
    }

    public void move(double power) {
        // SmartDashboard.putNumber("winch power", power);
        // winch.set(ControlMode.PercentOutput, power);
    }

    public void eject() {
        setPneumatics(DoubleSolenoid.Value.kForward);
    }

    public void uneject() {
        setPneumatics(DoubleSolenoid.Value.kReverse);
    }

    private void setPneumatics(DoubleSolenoid.Value value) {
        SmartDashboard.putNumber("setPneumatics called time", numCalls);
        numCalls++;
        for (int i = 0; i < numPneumatics; i++) {
            SmartDashboard.putBoolean("true = forward, false = reverse", (value == Value.kForward) ? true : false);
            pneumatics[i].set(value);
        }
    }
}

