// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
// import frc.robot.Commands.Shooter.BeltDrive;
// import frc.robot.Commands.Shooter.FineAdjust;
// import frc.robot.Commands.Shooter.Intake;
// import frc.robot.Commands.Shooter.SetPoint;
// import frc.robot.Commands.Shooter.Shoot;
// import frc.robot.Subsystems.Shooter.Arm;
// import frc.robot.Subsystems.Shooter.Shooter;
import frc.robot.Subsystems.Swerve.Swerve;
import frc.robot.Subsystems.Swerve.SwerveRequest;
import frc.robot.Subsystems.Swerve.SwerveModule.DriveRequestType;

public class RobotContainer {
	private double MaxSpeed = 6;
	private double MaxAngularRate = 1.5 * Math.PI;

	/* Setting up bindings for necessary control of the swerve drive platform */
	private final CommandXboxController xDrive = new CommandXboxController(0);
	private final CommandXboxController manip = new CommandXboxController(1);

	private final Swerve drivetrain = Swerve.getInstance();
	// public final Arm arm = Arm.getInstance();
	// public final Shooter belt = Shooter.getInstance();

	private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
			.withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1)
			.withDriveRequestType(DriveRequestType.OpenLoopVoltage);
	private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
	private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
			.withDriveRequestType(DriveRequestType.OpenLoopVoltage);
	private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

	/* Path follower */
	// private Command runAuto = drivetrain.getAutoPath("Tests");

	private final Telemetry logger = new Telemetry(MaxSpeed);

	private void configureBindings() {
		drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
				drivetrain.applyRequest(() -> drive.withVelocityX(-xDrive.getLeftY() * MaxSpeed) // Drive forward with
																									// negative Y
																									// (forward)
						.withVelocityY(-xDrive.getLeftX() * MaxSpeed) // Drive left with negative X (left)
						.withRotationalRate(-xDrive.getRightX() * MaxAngularRate) // Drive counterclockwise with
																					// negative X (left)
				).ignoringDisable(true));

		xDrive.a().whileTrue(drivetrain.applyRequest(() -> brake));
		xDrive.b().whileTrue(drivetrain
				.applyRequest(() -> point.withModuleDirection(new Rotation2d(-xDrive.getLeftY(), -xDrive.getLeftX()))));

		// reset the field-centric heading on left bumper press
		xDrive.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

		if (Robot.isSimulation()) {
			drivetrain.seedFieldRelative(new Pose2d(new Translation2d(),
					Rotation2d.fromDegrees(90)));
		}
		
		drivetrain.registerTelemetry((telemetry) -> logger.telemeterize(telemetry));

		xDrive.pov(0).whileTrue(drivetrain.applyRequest(() -> forwardStraight.withVelocityX(0.5).withVelocityY(0)));
		xDrive.pov(180).whileTrue(drivetrain.applyRequest(() -> forwardStraight.withVelocityX(-0.5).withVelocityY(0)));

		// arm.setDefaultCommand(
		// 	new FineAdjust(() -> manip.getLeftY())
		// );
				
		// belt.setDefaultCommand(
		// 	new BeltDrive()
		// );

		// manip.a().onTrue(new SetPoint(140).withTimeout(2));
		// manip.b().onTrue(new SetPoint(10).withTimeout(2));
		// manip.x().whileTrue(new Intake());
		// manip.y().whileTrue(new Shoot());

	}

	public RobotContainer() {
		configureBindings();
	}

	public Command getAutonomousCommand() {
		/* First put the drivetrain into auto run mode, then run the auto */
		return new PrintCommand("No auto loaded");
	}
}