// package frc.robot.Subsystems.Shooter;

// import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
// import com.ctre.phoenix6.configs.Slot0Configs;
// import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
// import com.ctre.phoenix6.controls.PositionDutyCycle;
// import com.ctre.phoenix6.controls.VelocityDutyCycle;
// import com.ctre.phoenix6.signals.ForwardLimitSourceValue;
// import com.ctre.phoenix6.signals.GravityTypeValue;
// import com.ctre.phoenix6.signals.ReverseLimitSourceValue;

// import edu.wpi.first.math.util.Units;
// import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
// import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj.util.Color;
// import edu.wpi.first.wpilibj.util.Color8Bit;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants;
// import frc.robot.Robot;
// import frc.robot.Custom.LoggyThings.LoggyCANcoder;
// import frc.robot.Custom.LoggyThings.LoggyTalonFX;

// public class Arm extends SubsystemBase {
//     private static Arm mInstance;
//     private LoggyTalonFX armMotor;
//     private LoggyCANcoder armCoder;
//     private PositionDutyCycle armControl;
//     private MotionMagicDutyCycle armMotionControl;
//     private VelocityDutyCycle armVelocityControl;
//     private double armRotation;

//     private Mechanism2d armMechanism;
//     private MechanismLigament2d elbowLigament, shoulderLigament, phantomLigament;

//     public static Arm getInstance() {
//         if (mInstance == null) {
//             mInstance = new Arm();
//         }

//         return mInstance;
//     }

//     private Arm() {
//         armControl = new PositionDutyCycle(0).withEnableFOC(true).withFeedForward(Constants.ArmConstants.kFeedForward)
//                 .withUpdateFreqHz(20).withVelocity(2);

//         armVelocityControl = new VelocityDutyCycle(0).withEnableFOC(true);
//         armMotionControl = new MotionMagicDutyCycle(0);

//         armMotor = new LoggyTalonFX(Constants.ArmConstants.armMotorID, false);
//         armMotor.setControl(armControl.withFeedForward(1).withPosition(0.5));
//         armMotor.setPosition(Units.degreesToRotations(armRotation));
//         Slot0Configs configs = new Slot0Configs();
//         configs.kP = 0.5;
//         configs.kI = 0;
//         configs.kD = 0;
//         configs.GravityType = GravityTypeValue.Arm_Cosine;
//         armMotor.setSlotConfig(configs);

//         armCoder = new LoggyCANcoder(Constants.ArmConstants.armEncoderID, false);

//         armMotor.setSoftLimit(
//                 new HardwareLimitSwitchConfigs()
//                         .withForwardLimitSource(ForwardLimitSourceValue.RemoteCANcoder)
//                         .withForwardLimitRemoteSensorID(Constants.ArmConstants.armEncoderID)
//                         .withForwardLimitEnable(true)
//                         .withForwardLimitAutosetPositionValue(Constants.ArmConstants.kArmMinAngle)
//                         .withReverseLimitSource(ReverseLimitSourceValue.RemoteCANcoder)
//                         .withReverseLimitRemoteSensorID(Constants.ArmConstants.armEncoderID)
//                         .withReverseLimitEnable(true)
//                         .withReverseLimitAutosetPositionValue(Constants.ArmConstants.kArmMaxAngle));

//         /* Robot is 30 inches front to back, bumper to bumper */
//         double bumperToBumper = 15, tolerance = 10;

//         armRotation = 90;
//         armMechanism = new Mechanism2d(2 * Units.inchesToMeters(bumperToBumper + tolerance), Units.feetToMeters(2.5),
//                 new Color8Bit(Color.kOrange));
//         elbowLigament = new MechanismLigament2d("Elbow", Units.inchesToMeters(bumperToBumper), armRotation, 18,
//                 new Color8Bit(Color.kBlack));
//         shoulderLigament = new MechanismLigament2d("Shoulder", Units.feetToMeters(0.35), -90, 18,
//                 new Color8Bit(Color.kBlack));
//         phantomLigament = new MechanismLigament2d("Phantom", Units.feetToMeters(2), 60, 9, new Color8Bit(Color.kGray));
//         armMechanism.getRoot("Root", Units.inchesToMeters(bumperToBumper + tolerance), Units.feetToMeters(0.5))
//                 .append(elbowLigament).append(shoulderLigament).append(phantomLigament);
//         SmartDashboard.putData("Mechanism", armMechanism);
//     }

//     /**
//      * Sets the arm to a position in rotations
//      * 
//      * @param position position in rotations
//      */
//     public void setArm(double position) {
//         double rotations = Units.degreesToRotations(position);
//         armMotor.setControl(armMotionControl.withPosition(rotations).withSlot(0));
//         // armMotor.setControl(armControl.withPosition(Units.degreesToRotations(position)));
//         armRotation = position;
//     }

//     /** Still need to write code but will aim arm at the slot using vision */
//     public void aim() {
//         /* TODO: Set up an algorithm to aim for slot/apriltag */
//     }

//     public double inRange(double position) {
//         if (position > Constants.ArmConstants.kArmMinAngle) {
//             return getArmRotation();
//         } else if (position < Constants.ArmConstants.kArmMaxAngle) {
//             return getArmRotation();
//         } else {
//             return position;
//         }
//     }

//     public void setArmVelocity(double velocity) {
//         armMotor.setControl(armVelocityControl.withVelocity(velocity));
//         armRotation += velocity;
//     }

//     public double getArmVelocity() {
//         return armMotor.getVelocity().getValue();
//     }

//     @Override
//     public void periodic() {
//         elbowLigament.setAngle(getArmRotation());
//     }

//     public double getArmRotation() {
//         if (Robot.isSimulation()) {
//             return armRotation;
//         } else {
//             return armCoder.getPosition().getValue();
//         }
//     }
// }
