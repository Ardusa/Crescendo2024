package frc.robot.Subsystems.Shooter;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Custom.LoggyThings.LoggyCANcoder;
import frc.robot.Custom.LoggyThings.LoggyTalonFX;

/**
 * To zero, first put the arm in a position where the shooter extension bars are parrallel to the long shooter bars
 * Zero the armCoder and then write enter the Magnet Offset into line 61
 * 
 */
public class Arm extends SubsystemBase {
    private static Arm mInstance;
    private LoggyTalonFX armMotor;
    private LoggyCANcoder armCoder;
    private double targetPosition = 0;

    private Mechanism2d armMechanism;
    private MechanismLigament2d shooterLigament, shooterExtension, elbowLigament;

    private Mechanism2d simArmMechanism;
    private MechanismLigament2d simShooterLigament, simShooterExtension, simElbowLigament;

    public Runnable feedForward = () -> {
        armMotor.set(0.1);
    };

    // private final double shooterOffset = -1.4;

    public static Arm getInstance() {
        if (mInstance == null) {
            mInstance = new Arm();
        }

        return mInstance;
    }

    @Override
    public void periodic() {
        if (Robot.isReal()) {
            shooterExtension.setAngle(getShooterExtensionPosition());
        }
        // System.out.println("Target: " + targetPosition);
        simShooterExtension.setAngle(targetPosition - 90);
    }

    private Arm() {
        armCoder = new LoggyCANcoder(Constants.ArmConstants.armEncoderID, false);
        CANcoderConfiguration armCoderConfig = new CANcoderConfiguration();

        /*
         * Do this in code so that it doesnt have to be done when cancoder gets replaced
         */
        armCoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
        armCoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        armCoderConfig.MagnetSensor.MagnetOffset = -0.102783203125; // Try leaving this empty and seeing if it works automatically
        armCoder.getConfigurator().apply(armCoderConfig);

        armMotor = new LoggyTalonFX(Constants.ArmConstants.armMotorID, false);
        TalonFXConfiguration armMotorConfig = new TalonFXConfiguration();
        armMotorConfig.Feedback.SensorToMechanismRatio = 2;
        armMotorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.SyncCANcoder;
        armMotorConfig.Feedback.RotorToSensorRatio = 50;
        armMotorConfig.Feedback.FeedbackRemoteSensorID = armCoder.getDeviceID();
        armMotor.getConfigurator().apply(armMotorConfig);
        armMotor.setNeutralMode(NeutralModeValue.Brake);

        // armControl = new PositionDutyCycle(getArmPosition()).withSlot(0);
        // armControl.withFeedForward(0.01);

        armMotor.getConfigurator().apply(new Slot0Configs().withKP(0.1).withKI(0).withKD(0));

        double shooterHeightInches = 22;
        double shooterLengthInches = 9.5;
        double shooterExtensionInches = 4.5;
        double shooterExtensionAngle = -90;
        double elbowLigamentAngle = -58.6;
        double rootXInches = Units.metersToInches(0.7493);
        double rootYInches = 4;

        if (Robot.isReal()) {
            /* Motor Profile */
            armMechanism = new Mechanism2d(0.7493 * 2, Units.feetToMeters(3));

            shooterLigament = new MechanismLigament2d("Shooter Bars", Units.inchesToMeters(shooterHeightInches), 90, 9,
                    new Color8Bit(Color.kWhite));

            shooterExtension = new MechanismLigament2d("Shooter Extension",
                    Units.inchesToMeters(shooterExtensionInches), shooterExtensionAngle, 2, new Color8Bit(Color.kGray));

            elbowLigament = new MechanismLigament2d("Shooter", Units.inchesToMeters(shooterLengthInches),
                    elbowLigamentAngle, 5, new Color8Bit(Color.kWhite));

            armMechanism.getRoot("Root", Units.inchesToMeters(rootXInches), Units.inchesToMeters(rootYInches))
                    .append(shooterLigament).append(shooterExtension).append(elbowLigament);
            SmartDashboard.putData("Arm Motor Profile", armMechanism);
        }


        /* Create Target Mechanism */
        simArmMechanism = new Mechanism2d(0.7493 * 2, Units.feetToMeters(3));
        
        simShooterLigament = new MechanismLigament2d("Shooter Bars", Units.inchesToMeters(shooterHeightInches), 90, 9,
                new Color8Bit(Color.kRed));

        simShooterExtension = new MechanismLigament2d("Shooter Extension", Units.inchesToMeters(shooterExtensionInches),
                shooterExtensionAngle, 2, new Color8Bit(Color.kDarkRed));

        simElbowLigament = new MechanismLigament2d("Shooter", Units.inchesToMeters(shooterLengthInches),
                elbowLigamentAngle, 5, new Color8Bit(Color.kRed));

        simArmMechanism.getRoot("Root", Units.inchesToMeters(rootXInches), Units.inchesToMeters(rootYInches))
                .append(simShooterLigament).append(simShooterExtension).append(simElbowLigament);
        SmartDashboard.putData("Arm Target Profile", simArmMechanism);
    }

    // /** Still need to write code but will aim arm at the slot using vision */
    // public void aim() {
    //     /* TODO: Set up an algorithm to aim for slot/apriltag */
    //     /* look into this
    //     new ArmFeedforward(0, 0, 0).
    //     */
    // }

    /**
     * 
     * 
     * @param target target position in degrees
     * @return true if in range or false if out of range
     */
    public boolean isInRangeOfTarget(double target) {
        if (Math.abs(getArmPosition() - target) < 5) {
            return true;
        } else {
            return false;
        }
    }

    /**
     * Whether or not the SHOOTER is within the allowable range of motion
     * @param setpoint the Shooter setpoint in degrees
     * @return true if within range, false if not
     */
    public boolean validSetpoint(double setpoint) {
        if (setpoint > Constants.ArmConstants.kArmMinAngle && setpoint < Constants.ArmConstants.kArmMaxAngle) {
            return true;
        } else {
            return false;
        }
    }

    /**
     * supply a specific value [-1,1] to the arm motor
     * @param setVal the value to set the arm motor to [-1,1]
     */
    public void aim(double setVal) {        
        if (Math.abs(setVal) <= 0.06) {
            setVal = 0.02;
        }

        if (!validSetpoint((5 * setVal) + getArmPosition())) {
            setVal = 0;
        }

        armMotor.set(setVal);
    }

    /**
     * Returns the position of the arm in degrees
     * 
     * @return position in degrees
     */
    public double getArmPosition() {
        if (Robot.isSimulation()) {
            return simShooterExtension.getAngle() - Constants.ArmConstants.shooterOffset + 90;
        }
        return (armCoder.getAbsolutePosition().getValueAsDouble() / 2) - Constants.ArmConstants.shooterOffset + 90;
    }

    public double getShooterExtensionPosition() {
        return getArmPosition() + Constants.ArmConstants.shooterOffset;
    }

    public void stop() {
        armMotor.set(0);
    }

    /**
     * Sets the Sim Arm Target Mechanism to a specific position
     * @param target in degrees
     */
    public void setArmTarget(double target) {
        targetPosition = target;
    }
}
