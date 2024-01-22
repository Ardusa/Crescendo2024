package frc.robot;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstantsFactory;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.ClosedLoopOutputType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants.SteerFeedbackType;
import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

//Numbering system for drivetrain: 0 - front right, 1 - front left, 2 - back left, 3 - back right
//0.42545 + 0.254/2

public final class Constants {
	/** Should be 16.54 */
	public static final double fieldLength = Units.inchesToMeters(76.1 + 250.5) * 2;
	public static final double fieldHeight = 8.21;

	public static final double kRange = 20;

	public static final boolean UseLimelight = true;

	public class SwerveConstants {
		public static final double kMaxSpeedMetersPerSecond = 6;
		public static final double kMaxAngularSpeedMetersPerSecond = 5 * Math.PI;

		// The steer motor uses any SwerveModule.SteerRequestType control request with
		// the output type specified by SwerveModuleConstants.SteerMotorClosedLoopOutput
		private static final Slot0Configs steerGains = new Slot0Configs()
				.withKP(100).withKI(0).withKD(0.2)
				.withKS(0).withKV(1.5).withKA(0);
		// When using closed-loop control, the drive motor uses the control
		// output type specified by SwerveModuleConstants.DriveMotorClosedLoopOutput
		private static final Slot0Configs driveGains = new Slot0Configs()
				.withKP(3).withKI(0).withKD(0)
				.withKS(0).withKV(0).withKA(0);

		// The closed-loop output type to use for the steer motors
		// This affects the PID/FF gains for the steer motors
		private static final ClosedLoopOutputType steerClosedLoopOutput = ClosedLoopOutputType.Voltage;
		// The closed-loop output type to use for the drive motors;
		// This affects the PID/FF gains for the drive motors
		private static final ClosedLoopOutputType driveClosedLoopOutput = ClosedLoopOutputType.Voltage;

		// The stator current at which the wheels start to slip;
		private static final double kSlipCurrentA = 300.0;

		// Theoretical free speed (m/s) at 12v applied output;
		public static final double kSpeedAt12VoltsMetersPerSecond = 5.13;

		// Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns;
		private static final double kCoupleRatio = 3.5714285714285716;

		private static final double kDriveGearRatio = 6.746031746031747;
		private static final double kSteerGearRatio = 21.428571428571427;
		private static final double kWheelRadiusInches = 4;
		// Estimated at first, then fudge-factored to make odom
		// match record

		private static final boolean kSteerMotorReversed = true;
		private static final boolean kInvertLeftSide = false;
		private static final boolean kInvertRightSide = true;

		// These are only used for simulation
		private static final double kSteerInertia = 0.00001;
		private static final double kDriveInertia = 0.001;
		// Simulated voltage necessary to overcome friction
		private static final double kSteerFrictionVoltage = 0.25;
		private static final double kDriveFrictionVoltage = 0.25;

		private static final SwerveModuleConstantsFactory ConstantCreator = new SwerveModuleConstantsFactory()
				.withDriveMotorGearRatio(kDriveGearRatio)
				.withSteerMotorGearRatio(kSteerGearRatio)
				.withWheelRadius(kWheelRadiusInches)
				.withSlipCurrent(kSlipCurrentA)
				.withSteerMotorGains(steerGains)
				.withDriveMotorGains(driveGains)
				.withSteerMotorClosedLoopOutput(steerClosedLoopOutput)
				.withDriveMotorClosedLoopOutput(driveClosedLoopOutput)
				.withSpeedAt12VoltsMps(kSpeedAt12VoltsMetersPerSecond)
				.withSteerInertia(kSteerInertia)
				.withDriveInertia(kDriveInertia)
				.withSteerFrictionVoltage(kSteerFrictionVoltage)
				.withDriveFrictionVoltage(kDriveFrictionVoltage)
				.withFeedbackSource(SteerFeedbackType.FusedCANcoder)
				.withCouplingGearRatio(kCoupleRatio)
				.withSteerMotorInverted(kSteerMotorReversed);

		/* Picture the front of the robot facing to the right in the XY axis */

		/** Distance between the 2 right side CANcoders */
		public static final double driveBaseWidth = 24.5;
		/** Distance between the 2 front side CANcoders */
		public static final double driveBaseHeight = 24;

		/**
		 * distance from the center of the robot to the furthest module (meters) should
		 * be 34.3
		 */
		public static final double driveBaseRadius = Utils.pythagorean(driveBaseWidth / 2, driveBaseHeight / 2);

		private static final String kCANbusName = "*";
		private static final int kPigeonId = 0;
		public static final SwerveDrivetrainConstants DrivetrainConstants = new SwerveDrivetrainConstants()
				.withPigeon2Id(kPigeonId)
				.withCANbusName(kCANbusName);

		// Front Left
		private static final int kFrontLeftDriveMotorId = 12;
		private static final int kFrontLeftSteerMotorId = 11;
		private static final int kFrontLeftEncoderId = 10;
		private static final double kFrontLeftEncoderOffset = -0.49658203125;
		private static final double kFrontLeftXPosInches = driveBaseWidth / 2;
		private static final double kFrontLeftYPosInches = driveBaseHeight / 2;

		// Front Right
		private static final int kFrontRightDriveMotorId = 22;
		private static final int kFrontRightSteerMotorId = 21;
		private static final int kFrontRightEncoderId = 20;
		private static final double kFrontRightEncoderOffset = 0.442138671875;
		private static final double kFrontRightXPosInches = driveBaseWidth / 2;
		private static final double kFrontRightYPosInches = -driveBaseHeight / 2;

		// Back Left
		private static final int kBackLeftDriveMotorId = 32;
		private static final int kBackLeftSteerMotorId = 31;
		private static final int kBackLeftEncoderId = 30;
		private static final double kBackLeftEncoderOffset = -0.34814453125;
		private static final double kBackLeftXPosInches = -driveBaseWidth / 2;
		private static final double kBackLeftYPosInches = driveBaseHeight;

		// Back Right
		private static final int kBackRightDriveMotorId = 52;
		private static final int kBackRightSteerMotorId = 51;
		private static final int kBackRightEncoderId = 40;
		private static final double kBackRightEncoderOffset = -0.076171875;
		private static final double kBackRightXPosInches = -driveBaseWidth / 2;
		private static final double kBackRightYPosInches = -driveBaseHeight / 2;

		public static final SwerveModuleConstants FrontLeft = ConstantCreator.createModuleConstants(
				kFrontLeftSteerMotorId, kFrontLeftDriveMotorId, kFrontLeftEncoderId, kFrontLeftEncoderOffset,
				Units.inchesToMeters(kFrontLeftXPosInches), Units.inchesToMeters(kFrontLeftYPosInches),
				kInvertLeftSide);

		public static final SwerveModuleConstants FrontRight = ConstantCreator.createModuleConstants(
				kFrontRightSteerMotorId, kFrontRightDriveMotorId, kFrontRightEncoderId,
				kFrontRightEncoderOffset, Units.inchesToMeters(kFrontRightXPosInches),
				Units.inchesToMeters(kFrontRightYPosInches), kInvertRightSide);
		public static final SwerveModuleConstants BackLeft = ConstantCreator.createModuleConstants(
				kBackLeftSteerMotorId, kBackLeftDriveMotorId, kBackLeftEncoderId, kBackLeftEncoderOffset,
				Units.inchesToMeters(kBackLeftXPosInches), Units.inchesToMeters(kBackLeftYPosInches),
				kInvertLeftSide);
		public static final SwerveModuleConstants BackRight = ConstantCreator.createModuleConstants(
				kBackRightSteerMotorId, kBackRightDriveMotorId, kBackRightEncoderId, kBackRightEncoderOffset,
				Units.inchesToMeters(kBackRightXPosInches), Units.inchesToMeters(kBackRightYPosInches),
				kInvertRightSide);
	}

	public enum AprilTag {
		NoTag(-1),
		BlueRightHumanPlayer(1),
		BlueLeftHumanPlayer(2),
		RedSpeakerOffset(3),
		RedSpeaker(4),
		RedAmp(5),
		BlueAmp(6),
		BlueSpeaker(7),
		BlueSpeakerOffset(8),
		RedRightHumanPlayer(9),
		RedLeftHumanPlayer(10),
		RedLeftStage(11),
		RedRightStage(12),
		RedCenterStage(13),
		BlueCenterStage(14),
		BlueLeftStage(15),
		BlueRightStage(16);

		public final int id;

		private AprilTag(int id) {
			this.id = id;
		}

		public static AprilTag fromId(int id) {
			for (AprilTag tag : AprilTag.values()) {
				if (tag.id == id) {
					return tag;
				}
			}
			return NoTag;
		}
	}

	public static class CustomDeadzone {

		public static final double kLowerLimitExpFunc = 0.1;
		public static final double kUpperLimitExpFunc = 0.5;
		public static final double kUpperLimitLinFunc = 1;

		public static final double kExpFuncConstant = 0.3218;
		public static final double kExpFuncBase = 12.5;
		public static final double kExpFuncMult = 0.25;

		public static final double kLinFuncMult = 0.876;
		public static final double kLinFuncOffset = 0.5;
		public static final double kLinFuncConstant = 0.562;

		public static final double kNoSpeed = 0;

		public static final double kJoyStickDeadZone = 0.05;
	}

	public static final class AutoConstants {

		public static final TrajectoryConfig config = new TrajectoryConfig(
				Constants.AutoConstants.kMaxSpeedMetersPerSecond,
				Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared);

		public static final PIDConstants translationPID = new PIDConstants(10, 0, 0);
		public static final PIDConstants rotationPID = new PIDConstants(10, 0, 0);
		public static final double kMaxSpeedMetersPerSecond = 3;
		public static final double kMaxAccelerationMetersPerSecondSquared = 5;
		public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
		public static final double kMaxAngularSpeedRadiansPerSecondSquared = 2 * Math.PI;

		public static final double kPXController = 10;
		public static final double kPYController = 10;
		public static final double kPThetaController = 1;

		public static final double intakeBeltOnTimeSeconds = 0.5;
		public static final double intakeDeployTimeSeconds = 0.5;

		/* Constraint for the motion profilied robot angle controller */
		public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
				kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);

		public static final String kFieldObjectName = "path";
	}

	public static class motorConstants {
		public static final double falconFreeSpeedRPM = 6380.0;

		/* Kraken x60 Info */
		public static class Kraken {
			public static final double krakenFreeSpeedRotationPerMinute = 5800.0;
			public static final double krakenFreeSpeedRadiansPerSecond = krakenFreeSpeedRotationPerMinute * 2 * Math.PI
					/ 60;
			public static final double krakenStallTorqueNM = 9.37;
			public static final double krakenStallCurrentAmps = 483;
			public static final double krakenPeakPowerWatts = 1405;
			public static final double krakenMaxEfficiencyWattsInOverWattsOut = 0.854;
			public static final double krakenCurrMaxEfficiencyAmps = 37;
		}
	}

	public static class OperatorConstants {
		public static final int kDriverControllerPort = 0;

		public static final double deadband = SwerveConstants.kMaxSpeedMetersPerSecond * 0.1;
		public static final double rotationalDeadband = SwerveConstants.kMaxAngularSpeedMetersPerSecond * 0.1;
	}

	public static class BeltConstants {
		public static final int beltMotorLeft = 60;
		public static final int beltMotorRight = 61;
		public static final boolean intakeIsPositive = true;

		public static final double kBeltSpeedSpeaker = 1;
		public static final double kBeltSpeedAmp = 0.2;
		public static final double kBeltIntakeSpeed = 0.2;

		public static final double kBeltVelocitySpeaker = 1;
		public static final double kBeltVelocityAmp = 0.2;
		public static final double kBeltIntakeVelocityMax = 94;
		public static final double kBeltIntakeVelocity20Percent = 94 * 0.2;
		public static final double kBeltFeedForward = 0.05;

		public static final double beltBufferVelocity = 10;
	}

	public static class ArmConstants {
		public static final int armMotorID = 60;
		public static final int armEncoderID = 61;

		public static final double kFeedForward = 0.5;

		public static final double kArmMaxAngle = 0;
		public static final double kArmMinAngle = 150;

		public static class SetPoints {
			public static final double kIntakeAngle = 0;
			public static final double kShootAngle = 30;
		}
	}

	public static class Lights {
		public static final double blinkTime = 7.5;
		public static final int blinkinPWM_ID = 0;
		public static final double kConeStatic = 0.11;
		public static final double kConeBlink = 0.15;
		public static final double kCubeStatic = 0.31;
		public static final double kCubeBlink = 0.35;
		public static final double kFireTwinkle = -0.49;
		public static final double kRobostangs = 0.63;
		public static final double kKillLights = 0.99;
	}
}
