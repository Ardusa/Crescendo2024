// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Subsystems.Swerve.Swerve;
import frc.robot.Subsystems.Swerve.SwerveRequest;

public class RobotContainer {

	/* Setting up bindings for necessary control of the swerve drive platform */
	private final CommandXboxController xDrive = new CommandXboxController(0);
	// private final CommandXboxController manip = new CommandXboxController(1);
	private final GenericHID simController = new GenericHID(3);

	private final Swerve drivetrain = Swerve.getInstance();
	// public final Arm arm = Arm.getInstance();
	// public final Shooter belt = Shooter.getInstance();

	private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
	private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
			.withDeadband(Constants.OperatorConstants.deadband)
			.withRotationalDeadband(Constants.OperatorConstants.rotationalDeadband);

	private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric();

	private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

	private final Telemetry logger;

	private void configureBindings() {
		drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
				drivetrain.applyRequest(() -> drive
						.withVelocityX(-xDrive.getLeftY() * Constants.SwerveConstants.kMaxSpeedMetersPerSecond)
						.withVelocityY(-xDrive.getLeftX() * Constants.SwerveConstants.kMaxSpeedMetersPerSecond)
						.withRotationalRate(
								-xDrive.getRightX() * Constants.SwerveConstants.kMaxAngularSpeedMetersPerSecond)
						.withSlowDown(xDrive.rightBumper().getAsBoolean(), 0.5)

				// negative X (left)
				).ignoringDisable(true));

		xDrive.a().whileTrue(drivetrain.applyRequest(() -> brake));
		xDrive.b().whileTrue(drivetrain
				.applyRequest(() -> point.withModuleDirection(new Rotation2d(-xDrive.getLeftY(), -xDrive.getLeftX()))));

		// reset the field-centric heading on left bumper press
		xDrive.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

		// xDrive.x().whileTrue(new AprilTagCmd());



		/* Set Sim Binds */
		// drivetrain.setDefaultCommand(
		// 		drivetrain.applyRequest(() -> drive.withVelocityX(simController.getRawAxis(0))
		// 				.withVelocityY(simController.getRawAxis(1)).withRotationalRate(simController.getRawAxis(2))));

		// new Trigger(() -> simController.getRawButtonPressed(1)).onTrue(
		// 		drivetrain.applyRequest(() -> brake));

		// new Trigger(() -> simController.getRawButtonPressed(2)).onTrue(
		// 		new InstantCommand(() -> drivetrain.seedFieldRelative()));

		// new Trigger(() -> simController.getRawButtonPressed(3)).whileTrue(
		// 		drivetrain.applyRequest(() -> drive.withVelocityX(1).withVelocityY(1)));
	}

	public RobotContainer() {
		logger = new Telemetry();
		drivetrain.registerTelemetry((telemetry) -> logger.telemeterize(telemetry));
		configureBindings();
	}

	public Command getAutonomousCommand() {
		/* First put the drivetrain into auto run mode, then run the auto */
		/* Path follower, Autobuilder already configured */
		// Command runAuto = drivetrain.getAutoPath("Tests");

		return new PrintCommand("No auto loaded");
	}
}