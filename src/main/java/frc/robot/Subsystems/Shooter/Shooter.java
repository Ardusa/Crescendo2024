package frc.robot.Subsystems.Shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Custom.LoggyThings.LoggyTalonFX;

public class Shooter extends SubsystemBase {
    private static Shooter mInstance;
    /** shootMotorRight is the master motor */
    private LoggyTalonFX shootMotorRight, shootMotorLeft, feedMotor;
    private VelocityVoltage shootPid = new VelocityVoltage(0);

    private boolean holding;

    public static Shooter getInstance() {
        if (mInstance == null) {
            mInstance = new Shooter();
        }

        return mInstance;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("left/actual Rpm", shootMotorLeft.getVelocity().getValueAsDouble() * 60);
        SmartDashboard.putNumber("right/actual Rpm", shootMotorRight.getVelocity().getValueAsDouble() * 60);

        SmartDashboard.putNumber("left/Supply Current", shootMotorLeft.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("right/Supply Current", shootMotorRight.getSupplyCurrent().getValueAsDouble());
    }

    private Shooter() {
        holding = false;

        shootMotorRight = new LoggyTalonFX(Constants.BeltConstants.shootMotorRight, false);
        shootMotorLeft = new LoggyTalonFX(Constants.BeltConstants.shootMotorLeft, false);

        TalonFXConfiguration fxConfig = new TalonFXConfiguration();
        fxConfig.CurrentLimits.SupplyCurrentLimit = 30;
        fxConfig.CurrentLimits.SupplyCurrentThreshold = 60;
        fxConfig.CurrentLimits.SupplyTimeThreshold = 0.5;
        fxConfig.MotorOutput.PeakReverseDutyCycle = 0;
        fxConfig.Slot0.kP = 0.2;
        fxConfig.Slot0.kI = 0.07;
        fxConfig.Slot0.kV = 2 / 16;
        fxConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        fxConfig.Feedback.SensorToMechanismRatio = 14 / 20;
        shootMotorLeft.getConfigurator().apply(fxConfig);
        shootMotorRight.getConfigurator().apply(fxConfig);

        feedMotor = new LoggyTalonFX(Constants.BeltConstants.feedMotor, false);
        feedMotor.setInverted(Constants.BeltConstants.feedIsInverted);
        
        feedMotor.setNeutralMode(NeutralModeValue.Coast);

        shootMotorRight.setInverted(Constants.BeltConstants.rightShootIsInverted);
        shootMotorLeft.setInverted(Constants.BeltConstants.leftShootIsInverted);

        SmartDashboard.putNumber("left/setRpm", 1000);
        SmartDashboard.putNumber("right/setRpm", 1000);
    }

    public void shoot(double shooter, double feeder) {
        shootMotorRight.set(shooter);
        shootMotorLeft.set(shooter);
        feedMotor.set(feeder);
    }

    public void stop() {
        shootMotorRight.set(0);
        shootMotorLeft.set(0);
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

    public void loadPiece() {
        feedMotor.set(1);
    }

    public boolean getSpeedChange() {
        if (shootMotorRight.getVelocity().getValueAsDouble() < Constants.MotorConstants.FalconRotorLoadThresholdRPM) {
            return true;
        }
        return false;
    }

    public void SetRpm(double left, double right) {
        shootMotorRight.setControl(shootPid.withVelocity(right / 60));
        shootMotorLeft.setControl(shootPid.withVelocity(left / 60));
    }

    public void setNeutralMode(NeutralModeValue mode) {
        shootMotorRight.setNeutralMode(mode);
        shootMotorLeft.setNeutralMode(mode);
    }
}