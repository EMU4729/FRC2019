package org.usfirst.frc4729.FRC2019.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import org.usfirst.frc4729.FRC2019.Util;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

    //private int numPneumatics = 2;
    private DoubleSolenoid[] pneumatics;
    // private boolean panelPistonIsOut;
    // private boolean mitchPistonIsUp;
    // private Compressor compressor;
    private DigitalInput limitTop;
    private DigitalInput limitBottom;
    private TalonSRX winch;
    private TalonSRX winch2;
    // private Encoder winchEncoder;

    public Mechanism() {
        // No Idea how to crossreference th wiring with what the code inputs should be 
        // TODO: Make sure that these i/os are actually right

    //     pneumatics = new DoubleSolenoid[numPneumatics];
    //     // for (int i = 0; i < numPneumatics; i++) {
    //         // pneumatics[i] = new DoubleSolenoid(i * 2, i * 2 + 1);
    //     // }
    //     // panel
    // //    pneumatics[0] = new DoubleSolenoid(3, 0, 1);
    //     // mitch
    // //    pneumatics[1] = new DoubleSolenoid(3, 6, 7); //TODO: Make sure these get used correctly. Currently not plugged in or attached

    //     pneumatics[0].set(DoubleSolenoid.Value.kForward);
    //     pneumatics[1].set(DoubleSolenoid.Value.kForward);
        
    //     panelPistonIsOut = true;
    //     mitchPistonIsUp = false;

    //     compressor = new Compressor(3);//TODO Check this numbers

        limitTop = new DigitalInput(0);      
        limitBottom = new DigitalInput(1);

        //winch = new PWMTalonSRX(1);
        //winch.setInverted(false);

        winch2 = new TalonSRX(5); // TODO change number later
        winch2.setNeutralMode(NeutralMode.Brake);

        winch = new TalonSRX(4);
        winch.setNeutralMode(NeutralMode.Brake);


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
       // updateWinch();
    }


    int[] levels = {100, 200, 300};
    int bottomPadding = levels[0] / 3;

    int slowRange = 50;
    int stopRange = 10;
    double minPower = 0.5;
    double maxPower = 1;

    // public void updateWinch() {
    //     double power;
    //     if (isAtLimitTop() || isAtLimitBottom()) {
    //         power = 0;
    //     } else {
    //         power = Util.linear(winchEncoder.get(),
    //                             levels[level],
    //                             minPower,
    //                             maxPower,
    //                             slowRange,
    //                             stopRange);
    //     }
    //     levelReached = (power == 0);
    //     if (winchEncoder.get() <= bottomPadding) power = 0.5;
    //     move(power);
    // }

    public void up() {
        winch.set(ControlMode.PercentOutput, 0.5);
        winch2.set(ControlMode.PercentOutput, 0.5);
      //  SmartDashboard.putBoolean("mechanism (should be) up", true);

        // if (level < levels.length - 1) {
        //     level++;
        // }
    }

    public void down() {
        winch.set(ControlMode.PercentOutput, -0.5);
        winch2.set(ControlMode.PercentOutput, -0.5);
       // SmartDashboard.putBoolean("mechanism (should be) down", false);
        // if (level > 0) {
        //     level--;
        // }
    }

    public void move(double power) {
        if (!winchCalibrating) {
            // winch.set(power);
        }
    }

    // public void eject() {
    //     setPneumatics(DoubleSolenoid.Value.kForward);
    // }

    public void stop() {
        winch.set(ControlMode.PercentOutput,0);
        winch2.set(ControlMode.PercentOutput,0);
    }

    // public void uneject() {
    //     pneumatics[0].set(DoubleSolenoid.Value.kReverse);
    // }

    // private void setPneumatics(DoubleSolenoid.Value value) {
    //     for (int i = 0; i < numPneumatics; i++) {
    //         // pneumatics[i].set(value);
    //     }
    // }


    // These two functions are not mapped to any control right now. TODO: Map them 
    // public void toggleMitch() {
    //     // This toggles the piston to lift/lower the panel frame
    //     if (mitchPistonIsUp) {
    //         pneumatics[0].set(DoubleSolenoid.Value.kReverse);
    //         mitchPistonIsUp = false;
            
    //     } else {
    //         pneumatics[0].set(DoubleSolenoid.Value.kForward);
    //         mitchPistonIsUp = true;
    //     }
    //     SmartDashboard.putBoolean("state of mitch", mitchPistonIsUp);
    // }
    // public void mitchHigh() {
    //     pneumatics[0].set(DoubleSolenoid.Value.kReverse);
    // }

    // public void mitchLow() {
    //     pneumatics[0].set(DoubleSolenoid.Value.kForward);
    // }

    // public void togglePanel() {
    //     // This toggles the piston to capture/release the panel mekanism
    //     if (panelPistonIsOut) {
    //         pneumatics[1].set(DoubleSolenoid.Value.kReverse);
    //         panelPistonIsOut = false;
    //     } else {
    //         pneumatics[1].set(DoubleSolenoid.Value.kForward);
    //         panelPistonIsOut = true;
    //     }
    // }

    // public void panelOut(){
    //     pneumatics[1].set(DoubleSolenoid.Value.kReverse);
    // }

    // public void panelIn(){
    //     pneumatics[1].set(DoubleSolenoid.Value.kForward);
    // }

    // public void resetWinchEncoder() {
    //     winchEncoder.reset();
    // }

    public boolean isAtLimitTop() {
        return false;//limitTop.get();
    }

    public boolean isAtLimitBottom() {
        return false;//limitBottom.get();
    }
}
