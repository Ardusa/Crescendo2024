package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Commands.Shooter.FineAdjust;
import frc.robot.Commands.Shooter.HoldToPosition;
import frc.robot.Commands.Shooter.SetPoint;
import frc.robot.Commands.Shooter.FeedAndShoot;
import frc.robot.Subsystems.Shooter.Arm;
import frc.robot.Subsystems.Shooter.Shooter;
import frc.robot.Subsystems.Swerve.Swerve;
import frc.robot.Subsystems.Swerve.SwerveRequest;
import frc.robot.Subsystems.Swerve.SwerveModule.DriveRequestType;

public class RobotContainer {

	/* Setting up bindings for necessary control of the swerve drive platform */
	private final CommandXboxController xDrive = new CommandXboxController(0);
	private final CommandXboxController Manip = new CommandXboxController(1);
	private final GenericHID simController = new GenericHID(3);

	private final Swerve drivetrain = Swerve.getInstance();
	private final Arm mArm = Arm.getInstance();
	private final Shooter mShooter = Shooter.getInstance();

	private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
	private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
			.withDeadband(Constants.OperatorConstants.deadband)
			.withRotationalDeadband(Constants.OperatorConstants.rotationalDeadband)
			.withDriveRequestType(DriveRequestType.OpenLoopVoltage);

	private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric();

	private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

	private final Telemetry logger;
	public Field2d field;

	private void configureBindings() {
		drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
				drivetrain.applyRequest(() -> drive
						.withVelocityX(-xDrive.getLeftY() * Constants.SwerveConstants.kMaxSpeedMetersPerSecond)
						.withVelocityY(-xDrive.getLeftX() * Constants.SwerveConstants.kMaxSpeedMetersPerSecond)
						.withRotationalRate(
								-xDrive.getRightX() * Constants.SwerveConstants.kMaxAngularSpeedMetersPerSecond)
						.withSlowDown(xDrive.getHID().getRightBumper(), 0.5)
				).ignoringDisable(true));

		xDrive.a().whileTrue(drivetrain.applyRequest(() -> brake));
		xDrive.b().whileTrue(drivetrain
				.applyRequest(() -> point.withModuleDirection(new Rotation2d(-xDrive.getLeftY(), -xDrive.getLeftX()))));

		xDrive.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

		xDrive.x().whileTrue(drivetrain.applyRequest(() -> forwardStraight.withVelocityX(1).withVelocityY(1)));

		// Manip.y().whileTrue(new SetPoint(Constants.ArmConstants.SetPoints.kAmp));
		// Manip.x().whileTrue(new SetPoint(Constants.ArmConstants.SetPoints.kSpeakerClosestPoint));

		Manip.y().whileTrue(new RunCommand(() -> mShooter.SetRpm(SmartDashboard.getNumber("left/setRpm", 0),
				SmartDashboard.getNumber("right/setRpm", 0))));

		Manip.b().whileTrue(new SetPoint(0));
		Manip.a().whileTrue(new SetPoint(Constants.ArmConstants.SetPoints.kSpeaker));

		Manip.leftBumper().whileTrue(new FeedAndShoot());

		mArm.setDefaultCommand(new FineAdjust(() -> -Manip.getRightY()));

		mShooter.setDefaultCommand(new RunCommand(() -> mShooter.stop(), mShooter));

		/* Set Sim Binds */
		if (Robot.isSimulation()) {
			drivetrain.setDefaultCommand(drivetrain
					.applyRequest(() -> drive
							.withVelocityX(
									-simController.getRawAxis(1) * Constants.SwerveConstants.kMaxSpeedMetersPerSecond)
							.withVelocityY(
									-simController.getRawAxis(0) * Constants.SwerveConstants.kMaxSpeedMetersPerSecond)
							.withRotationalRate(-simController.getRawAxis(3)
									* Constants.SwerveConstants.kMaxAngularSpeedMetersPerSecond / 2)
							.withSlowDown(simController.getRawButton(4), Constants.SwerveConstants.slowDownMultiplier))
					.withName("xDrive"));

			// new Trigger(() -> simController.getRawButtonPressed(1)).onTrue(drivetrain.applyRequest(() -> brake));

			new Trigger(() -> simController.getRawButtonPressed(1))
					.whileTrue(new HoldToPosition(Constants.ArmConstants.SetPoints.kSpeakerClosestPoint));

			new Trigger(() -> simController.getRawButtonPressed(2))
					.whileTrue(new HoldToPosition(Constants.ArmConstants.SetPoints.kAmp));

			new Trigger(() -> simController.getRawButtonPressed(3))
					.whileTrue(new HoldToPosition(Constants.ArmConstants.SetPoints.kSpeaker));

			new Trigger(() -> simController.getRawButtonPressed(4)).whileTrue(new HoldToPosition(0));

			// new Trigger(() -> simController.getRawButtonPressed(2))
			// 		.onTrue(new InstantCommand(() -> drivetrain.seedFieldRelative()));

			// new Trigger(() -> simController.getRawButtonPressed(3))
			// 		.onTrue(new SetPoint(Constants.ArmConstants.SetPoints.kSpeakerClosestPoint).withTimeout(1));

			// new Trigger(() -> simController.getRawButtonPressed(3)).onTrue(new SetPoint(0).withTimeout(1));

			// new Trigger(() -> simController.getRawButtonPressed(3))
			// 		.whileTrue(drivetrain.applyRequest(() -> forwardStraight.withVelocityX(1).withVelocityY(0)));

			mArm.setDefaultCommand(new FineAdjust(() -> -simController.getRawAxis(2)));

			// new Trigger(() -> simController.getRawButtonPressed(4)).onTrue(new InstantCommand(Intake.toggleDeploy));

		}
	}

	public RobotContainer() {
		logger = new Telemetry();
		field = Robot.mField;
		drivetrain.registerTelemetry((telemetry) -> logger.telemeterize(telemetry));
		configureBindings();
	}

	public Command getAutonomousCommand() {
		/* First put the drivetrain into auto run mode, then run the auto */
		return new PrintCommand("No auto loaded");
	}
}