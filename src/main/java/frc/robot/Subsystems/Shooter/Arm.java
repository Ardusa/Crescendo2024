package frc.robot.Subsystems.Shooter;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.ForwardLimitSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitSourceValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.controller.ArmFeedforward;
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

public class Arm extends SubsystemBase {
    private static Arm mInstance;
    private LoggyTalonFX armMotor;
    private TalonFXSimState armSim;
    private LoggyCANcoder armCoder;
    private CANcoderSimState armCoderSim;
    private PositionDutyCycle armControl;
    private ArmFeedforward feedforward;
    private double armRotation;
    private DoubleSupplier armPosition;

    private Mechanism2d armMechanism;
    private MechanismLigament2d shooterLigament, shooterExtension, elbowLigament;

    private final double shooterOffset = -33.3;

    public static Arm getInstance() {
        if (mInstance == null) {
            mInstance = new Arm();
        }

        return mInstance;
    }

    private Arm() {
        armCoder = new LoggyCANcoder(Constants.ArmConstants.armEncoderID, false);
        CANcoderConfiguration armCoderConfig = new CANcoderConfiguration();

        /*
         * Do this in code so that it doesnt have to be done when cancoder gets replaced
         */
        armCoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
        armCoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        armCoder.getConfigurator().apply(armCoderConfig);

        // armCoder.setPosition(90);

        armMotor = new LoggyTalonFX(Constants.ArmConstants.armMotorID, false);
        TalonFXConfiguration armMotorConfig = new TalonFXConfiguration();
        armMotorConfig.Feedback.SensorToMechanismRatio = 2;
        armMotorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.SyncCANcoder;
        armMotorConfig.Feedback.RotorToSensorRatio = 50;
        armMotorConfig.Feedback.FeedbackRemoteSensorID = armCoder.getDeviceID();
        armMotor.getConfigurator().apply(armMotorConfig);
        armMotor.setNeutralMode(NeutralModeValue.Brake);

        // armMotor.setSoftLimit(new HardwareLimitSwitchConfigs()
        //         .withForwardLimitSource(ForwardLimitSourceValue.RemoteCANcoder)
        //         .withForwardLimitRemoteSensorID(Constants.ArmConstants.armEncoderID).withForwardLimitEnable(true)
        //         .withForwardLimitAutosetPositionValue(Constants.ArmConstants.kArmMinAngle)
        //         .withReverseLimitSource(ReverseLimitSourceValue.RemoteCANcoder)
        //         .withReverseLimitRemoteSensorID(Constants.ArmConstants.armEncoderID).withReverseLimitEnable(true)
        //         .withReverseLimitAutosetPositionValue(Constants.ArmConstants.kArmMaxAngle));

        armPosition = () -> Units.rotationsToDegrees(armMotor.getPosition().getValue());
        armControl = new PositionDutyCycle(getArmPosition()).withSlot(0);

        armMotor.getConfigurator().apply(new Slot0Configs().withKP(0.1).withKI(0).withKD(0));

        armControl.withFeedForward(0.01);

        if (Robot.isSimulation()) {
            armSim = armMotor.getSimState();
            armCoderSim = armCoder.getSimState();
        }

        double shooterHeightInches = 22;
        double shooterLengthInches = 9.5;
        double shooterExtensionInches = 4.5;
        double shooterExtensionAngle = -90;
        double elbowLigamentAngle = -58.1;
        double rootXInches = 0.2;
        double rootYInches = 4;

        armMechanism = new Mechanism2d(0.7493 * 2, Units.feetToMeters(3));

        shooterLigament = new MechanismLigament2d("Shooter Bars", Units.inchesToMeters(shooterHeightInches), 90, 9,
                new Color8Bit(Color.kWhite));

        shooterExtension = new MechanismLigament2d("Shooter Extension", Units.inchesToMeters(shooterExtensionInches),
                shooterExtensionAngle, 2, new Color8Bit(Color.kGray));

        elbowLigament = new MechanismLigament2d("Shooter", Units.inchesToMeters(shooterLengthInches),
                elbowLigamentAngle, 5, new Color8Bit(Color.kWhite));

        armMechanism.getRoot("Root", Units.inchesToMeters(rootXInches), Units.inchesToMeters(rootYInches))
                .append(shooterLigament).append(shooterExtension).append(elbowLigament);
        SmartDashboard.putData("Arm Mechanism", armMechanism);

        setArmToPosition(-64);
    }

    // /** Still need to write code but will aim arm at the slot using vision */
    // public void aim() {
    //     /* TODO: Set up an algorithm to aim for slot/apriltag */
    //     /* look into this
    //     new ArmFeedforward(0, 0, 0).
    //     */
    // }

    /**
     * Returns the position of the arm in degrees
     * 
     * @param position target position in degrees
     * @return position in degrees or 0 if out of range
     * @deprecated soft limits are now set on the talon
     */
    public double inRange(double position) {
        if (position > Constants.ArmConstants.kArmMinAngle && position < Constants.ArmConstants.kArmMaxAngle) {
            return position;
        } else {
            return armRotation;
        }
    }

    /**
     * Returns the position of the arm in degrees
     * 
     * @param position target position in degrees
     * @return position in degrees or 0 if out of range
     */
    public boolean isInRangeOfTarget(double target) {
        if (Math.abs(Units.rotationsToDegrees(armRotation) - target) < 5) {
            return true;
        } else {
            return false;
        }

    }

    public void aim(double setVal) {
        if (Math.abs(setVal) <= 0.06) {
            setVal = 0;
        }

        armMotor.set(setVal);
    }

    @Override
    public void periodic() {
        if (Robot.isReal())
            shooterExtension.setAngle(armPosition.getAsDouble() - 90);
        else {
            shooterExtension.setAngle(Units.rotationsToDegrees(armRotation));
        }
    }

    /**
     * Sets the arm to a position in degrees
     * 
     * @param position position in degrees
     */
    public void setArmToPosition(double position) {
        double rotations = Units.degreesToRotations(position);
        System.out.println("Setting arm to " + position + " degrees");
        System.out.println("Setting arm to " + rotations + " rotations");
        position += shooterOffset;
        armRotation = Units.degreesToRotations(position);
        if (Robot.isSimulation()) {
            armSim.addRotorPosition(rotations);
        } else
            armMotor.setControl(armControl.withPosition(rotations));
    }

    /**
     * Returns the position of the arm in degrees
     * 
     * @return position in degrees
     */
    public double getArmPosition() {
        if (Robot.isSimulation()) {
            return Units.rotationsToDegrees(armRotation);
        }
        return Units.rotationsToDegrees(armMotor.getPosition().getValue());
    }
}
