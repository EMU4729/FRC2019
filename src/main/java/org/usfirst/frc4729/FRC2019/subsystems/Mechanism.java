package org.usfirst.frc4729.FRC2019.subsystems;
import org.usfirst.frc4729.FRC2019.commands.*;
import org.usfirst.frc4729.FRC2019.Util;

import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.PWMTalonSRX;
import edu.wpi.first.wpilibj.SpeedController;


public class Mechanism extends Subsystem {
    int level;

    private int numEjectPneumatics = 3;
    private DoubleSolenoid[] ejectPneumatics;
    private DoubleSolenoid extendPneumatic;
    private Compressor compressor;
    private DigitalInput limitTop;
    private DigitalInput limitBottom;
    private PWMTalonSRX winch1;
    private PWMTalonSRX winch2;
    private Encoder winchEncoder;


    public Mechanism() {
        ejectPneumatics = new DoubleSolenoid[numEjectPneumatics];
        for (int i = 0; i < numEjectPneumatics; i++) {
            ejectPneumatics[i] = new DoubleSolenoid(0, 0, 1);
            addChild("EjectPneumatic1",ejectPneumatics[i]);
        }
        
        extendPneumatic = new DoubleSolenoid(0, 6, 7);
        addChild("ExtendPneumatic",extendPneumatic);
        
        
        compressor = new Compressor(0);
        addChild("Compressor",compressor);
        
        
        limitTop = new DigitalInput(0);
        addChild("LimitTop",limitTop);
        
        limitBottom = new DigitalInput(1);
        addChild("LimitBottom",limitBottom);
        
        
        winch1 = new PWMTalonSRX(1);
        addChild("Winch1",winch1);
        winch1.setInverted(false);
        
        winch2 = new PWMTalonSRX(3);
        addChild("Winch2",winch2);
        winch2.setInverted(false);
        
        winchEncoder = new Encoder(2, 3, false, EncodingType.k4X);
        addChild("WinchEncoder",winchEncoder);
        winchEncoder.setDistancePerPulse(1.0);
        winchEncoder.setPIDSourceType(PIDSourceType.kDisplacement);
    }

    @Override
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        // setDefaultCommand(new MySpecialCommand());
    }

    @Override
    public void periodic() {
        updateWinch();
    }

    int[] levels = {100, 200, 300};
    int slowRange = 50;
    int stopRange = 10;
    double minPower = 0.5;
    double maxPower = 1;

    public void updateWinch() {
        if (limitTop.get() || limitBottom.get()) {
            move(0);
        } else {
            move(Util.linear(winchEncoder.get(), levels[level], minPower, maxPower, slowRange, stopRange));
        }
    }

    public void setLevel(int value) {
        level = value;
    }

    public void up() {
        if (level < levels.length - 1) {
            level++;
        }
    }

    public void down() {
        if (level > 0) {
            level--;
        }
    }

    private void move(double power) {
        winch1.set(power);
        winch2.set(power);
    }

    public void extend() {
        extendPneumatic.set(DoubleSolenoid.Value.kForward);
    }

    public void retract() {
        extendPneumatic.set(DoubleSolenoid.Value.kReverse);
    }

    public void eject() {
        setEjectPneumatics(DoubleSolenoid.Value.kForward);
    }

    public void uneject() {
        setEjectPneumatics(DoubleSolenoid.Value.kReverse);
    }

    private void setEjectPneumatics(DoubleSolenoid.Value value) {
        for (int i = 0; i < numEjectPneumatics; i++) {
            ejectPneumatics[i].set(value);
        }
    }
}

