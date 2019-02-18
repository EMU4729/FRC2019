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
    public int level = 0;
    public boolean levelReached = false;
    public boolean winchCalibrating = false;
    public boolean winchCalibrated = false;

    private int numPneumatics = 3;
    // private DoubleSolenoid[] pneumatics;
    // private Compressor compressor;
    // private DigitalInput limitTop;
    // private DigitalInput limitBottom;
    // private PWMTalonSRX winch;
    // private Encoder winchEncoder;

    public Mechanism() {
        // pneumatics = new DoubleSolenoid[numPneumatics];
        // for (int i = 0; i < numPneumatics; i++) {
        //     pneumatics[i] = new DoubleSolenoid(i * 2, i * 2 + 1);
        // }
        
        // compressor = new Compressor(0);

        // limitTop = new DigitalInput(0);      
        // limitBottom = new DigitalInput(1);

        // winch = new PWMTalonSRX(1);
        // winch.setInverted(false);

        // winchEncoder = new Encoder(2, 3, false, EncodingType.k4X);
        // winchEncoder.setDistancePerPulse(1.0);
        // winchEncoder.setPIDSourceType(PIDSourceType.kDisplacement);
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
    int bottomPadding = levels[0] / 3;

    int slowRange = 50;
    int stopRange = 10;
    double minPower = 0.5;
    double maxPower = 1;

    public void updateWinch() {
        // double power;
        // if (isAtLimitTop() || isAtLimitBottom()) {
        //     power = 0;
        // } else {
        //     power = Util.linear(winchEncoder.get(),
        //                         levels[level],
        //                         minPower,
        //                         maxPower,
        //                         slowRange,
        //                         stopRange);
        // }
        // levelReached = (power == 0);
        // if (winchEncoder.get() <= bottomPadding) power = 0.5;
        // move(power);
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

    public void move(double power) {
        if (!winchCalibrating) {
            // winch.set(power);
        }
    }

    public void eject() {
        setPneumatics(DoubleSolenoid.Value.kForward);
    }

    public void uneject() {
        setPneumatics(DoubleSolenoid.Value.kReverse);
    }

    private void setPneumatics(DoubleSolenoid.Value value) {
        for (int i = 0; i < numPneumatics; i++) {
            // pneumatics[i].set(value);
        }
    }

    public void resetWinchEncoder() {
        // winchEncoder.reset();
    }

    public boolean isAtLimitTop() {
        return false;//limitTop.get();
    }

    public boolean isAtLimitBottom() {
        return false;//limitBottom.get();
    }
}

