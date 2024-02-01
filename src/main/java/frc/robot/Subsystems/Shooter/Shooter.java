package frc.robot.Subsystems.Shooter;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Custom.LoggyThings.LoggyTalonFX;

public class Shooter extends SubsystemBase {
    private static Shooter mInstance;
    /** shootMotorRight is the master motor */
    private LoggyTalonFX shootMotorRight, shootMotorLeft, feedMotor;

    public enum Target {
        kSpeaker,
        kAmp,
        None
    }

    private boolean holding;

    public static Shooter getInstance() {
        if (mInstance == null) {
            mInstance = new Shooter();
        }

        return mInstance;
    }

    private Shooter() {
        shootMotorRight = new LoggyTalonFX(Constants.BeltConstants.shootMotorRight, false);
        shootMotorRight.setInverted(Constants.BeltConstants.rightShootIsInverted);

        shootMotorLeft = new LoggyTalonFX(Constants.BeltConstants.shootMotorLeft, false);
        shootMotorLeft.setInverted(Constants.BeltConstants.leftShootIsInverted);

        feedMotor = new LoggyTalonFX(Constants.BeltConstants.feedMotor, false);
        feedMotor.setInverted(Constants.BeltConstants.feedIsInverted);
        
        shootMotorRight.setNeutralMode(NeutralModeValue.Coast);
        shootMotorLeft.setNeutralMode(NeutralModeValue.Coast);
        feedMotor.setNeutralMode(NeutralModeValue.Coast);

        holding = false;
    }

    public void shoot(double shooter, double feeder) {
        shootMotorRight.set(shooter);
        feedMotor.set(feeder);
        shootMotorLeft.set(shooter);
    }

    public void stop() {
        shootMotorRight.set(0);
        feedMotor.set(0);
    }

    public void setHolding(boolean holding) {
        this.holding = holding;
    }

    public void toggleHolding() {
        holding = !holding;
    }

    public boolean getHolding() {
        return holding;
    }

    @Override
    public void periodic() {
        // System.out.println("Shooter alive");
        // if (Robot.isReal()) {
        //     SmartDashboard.putNumber("Velocity Left Belt", shootMotorRight.getVelocity().getValue());
        //     SmartDashboard.putNumber("Velocity Right Belt", shootMotorLeft.getVelocity().getValue());
        // }
    }
}