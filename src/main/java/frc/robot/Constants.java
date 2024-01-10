package frc.robot;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstantsFactory;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.ClosedLoopOutputType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants.SteerFeedbackType;

import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

//Numbering system for drivetrain: 0 - front right, 1 - front left, 2 - back left, 3 - back right
//0.42545 + 0.254/2

public final class Constants {
	/** Should be 16.6 */
	public static final double fieldLength = Units.inchesToMeters(76.1 + 250.5) * 2;

	public static final double kRange = 20;

	public class SwerveConstants {
		// TODO: Both sets of gains need to be tuned to your individual robot.

		// The steer motor uses any SwerveModule.SteerRequestType control request with
		// the output type specified by SwerveModuleConstants.SteerMotorClosedLoopOutput
		private static final Slot0Configs steerGains = new Slot0Configs()
				.withKP(100).withKI(0).withKD(0.05)
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
		// TODO: This needs to be tuned to your individual robot
		private static final double kSlipCurrentA = 300.0;

		// Theoretical free speed (m/s) at 12v applied output;
		// TODO: This needs to be tuned to your individual robot
		public static final double kSpeedAt12VoltsMps = 5.0;

		// Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns;
		// TODO: This may need to be tuned to your individual robot
		private static final double kCoupleRatio = 3.5;

		private static final double kDriveGearRatio = 7.363636364;
		private static final double kSteerGearRatio = 15.42857143;
		private static final double kWheelRadiusInches = 2.167; // Estimated at first, then fudge-factored to make odom
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
				.withSpeedAt12VoltsMps(kSpeedAt12VoltsMps)
				.withSteerInertia(kSteerInertia)
				.withDriveInertia(kDriveInertia)
				.withSteerFrictionVoltage(kSteerFrictionVoltage)
				.withDriveFrictionVoltage(kDriveFrictionVoltage)
				.withFeedbackSource(SteerFeedbackType.FusedCANcoder)
				.withCouplingGearRatio(kCoupleRatio)
				.withSteerMotorInverted(kSteerMotorReversed);

		/* Picture the robot facing to the right in the XY graph */

		/** X value */
		public static final double driveBaseWidth = 30;
		/** Y value */
		public static final double driveBaseHeight = 30;

		/** distance from the center of the robot to the furthest module (meters) */
		public static final double driveBaseRadius = Utils.pythagorean(driveBaseWidth / 2, driveBaseHeight / 2);

		private static final String kCANbusName = "*";
		private static final int kPigeonId = 0;
		public static final SwerveDrivetrainConstants DrivetrainConstants = new SwerveDrivetrainConstants()
				.withPigeon2Id(kPigeonId)
				.withCANbusName(kCANbusName);


		// Front Left
		private static final int kFrontLeftDriveMotorId = 01;
		private static final int kFrontLeftSteerMotorId = 02;
		private static final int kFrontLeftEncoderId = 03;
		private static final double kFrontLeftEncoderOffset = -0.83544921875;
		private static final double kFrontLeftXPosInches = driveBaseWidth / 2;
		private static final double kFrontLeftYPosInches = driveBaseHeight / 2;

		// Front Right
		private static final int kFrontRightDriveMotorId = 11;
		private static final int kFrontRightSteerMotorId = 12;
		private static final int kFrontRightEncoderId = 13;
		private static final double kFrontRightEncoderOffset = -0.15234375;
		private static final double kFrontRightXPosInches = driveBaseWidth / 2;
		private static final double kFrontRightYPosInches = -driveBaseHeight / 2;

		// Back Left
		private static final int kBackLeftDriveMotorId = 21;
		private static final int kBackLeftSteerMotorId = 22;
		private static final int kBackLeftEncoderId = 23;
		private static final double kBackLeftEncoderOffset = -0.4794921875;
		private static final double kBackLeftXPosInches = -driveBaseWidth / 2;
		private static final double kBackLeftYPosInches = driveBaseHeight;

		// Back Right
		private static final int kBackRightDriveMotorId = 31;
		private static final int kBackRightSteerMotorId = 32;
		private static final int kBackRightEncoderId = 33;
		private static final double kBackRightEncoderOffset = -0.84130859375;
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

		public static final double kMaxSpeedMetersPerSecond = 3;
		public static final double kMaxAccelerationMetersPerSecondSquared = 5;
		public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
		public static final double kMaxAngularSpeedRadiansPerSecondSquared = 2 * Math.PI;

		public static final double kPXController = 10;
		public static final double kPYController = 10;
		public static final double kPThetaController = 1;

		/* Constraint for the motion profilied robot angle controller */
		public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
				kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);

		public static final String kFieldObjectName = "path";
	}

	public static class motorConstants {
		public static final double falconFreeSpeedRPM = 6380.0;

		/* Kraken x60 Info */
		public static class Kraken {
			public static final double krakenFreeSpeedRPM = 5800.0;
			public static final double krakenFreeSpeedRadPerSec = krakenFreeSpeedRPM * 2 * Math.PI / 60;
			public static final double krakenStallTorqueNM = 9.37;
			public static final double krakenStallCurrentAmps = 483;
			public static final double krakenPeakPowerWatts = 1405;
			public static final double krakenMaxEfficiencyWattsInOverWattsOut = 0.854;
			public static final double krakenCurrMaxEfficiencyAmps = 37;
		}
	}

	public static class OperatorConstants {
		public static final int kDriverControllerPort = 0;
	}

	public static class BeltConstants {
		public static final int beltMotorID = 0;
		public static final boolean intakeIsPositive = true;

		public static final double kBeltSpeedSpeaker = 1;
		public static final double kBeltSpeedAmp = 0.2;
		public static final double kBeltIntakeSpeed = 0.2;

		public static final double kBeltVelocitySpeaker = 1;
		public static final double kBeltVelocityAmp = 0.2;
		public static final double kBeltIntakeVelocity = 0.2;
		public static final double kBeltFeedForward = 0.1;

	}

	public static class ArmConstants {
		public static final int armMotorID = 0;
		public static final int armEncoderID = 0;

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
