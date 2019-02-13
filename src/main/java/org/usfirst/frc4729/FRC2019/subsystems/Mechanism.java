package org.usfirst.frc4729.FRC2019.subsystems;

import org.usfirst.frc4729.FRC2019.Util;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.PWMTalonSRX;

public class Mechanism extends Subsystem {
    public int level;

    private int numPneumatics = 3;
    private DoubleSolenoid[] pneumatics;
    private Compressor compressor;
    private DigitalInput limitTop;
    private DigitalInput limitBottom;
    private PWMTalonSRX winch1;
    private PWMTalonSRX winch2;
    private Encoder winchEncoder;

    public Mechanism() {
        pneumatics = new DoubleSolenoid[numPneumatics];
        for (int i = 0; i < numPneumatics; i++) {
            pneumatics[i] = new DoubleSolenoid(0, 0, 1);
            addChild("EjectPneumatic1", pneumatics[i]);
        }
        
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
            move(Util.linear(winchEncoder.get(),
                             levels[level],
                             minPower,
                             maxPower,
                             slowRange,
                             stopRange));
        }
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

    public void eject() {
        setPneumatics(DoubleSolenoid.Value.kForward);
    }

    public void uneject() {
        setPneumatics(DoubleSolenoid.Value.kReverse);
    }

    private void setPneumatics(DoubleSolenoid.Value value) {
        for (int i = 0; i < numPneumatics; i++) {
            pneumatics[i].set(value);
        }
    }
}

