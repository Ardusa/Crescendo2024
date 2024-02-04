// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Commands.PathPlannerCommand;
import frc.robot.Subsystems.Shooter.Shooter;
import frc.robot.Subsystems.Swerve.Swerve;

public class Robot extends TimedRobot {
	public SendableChooser<String> mChooser = new SendableChooser<>();
	public static Field2d mField = new Field2d();

	public static boolean atComp = false;

	@Override
	public void robotInit() {
		
		new RobotContainer();

		SmartDashboard.putData("Field", mField);

		mChooser.setDefaultOption("Do Nothing", "null");
		AutoBuilder.getAllAutoNames().forEach((name) -> mChooser.addOption(name, name));
		SmartDashboard.putData("Auton Chooser", mChooser);

		// Swerve.getInstance().getDaqThread().setThreadPriority(99);

		CommandScheduler.getInstance()
				.onCommandInitialize((action) -> DataLogManager.log("\n" + action.getName() + " Command Initialized\n"));
		CommandScheduler.getInstance()
				.onCommandInterrupt((action) -> DataLogManager.log("\n" + action.getName() + " Command Interrupted\n"));
		CommandScheduler.getInstance()
				.onCommandFinish((action) -> DataLogManager.log("\n" + action.getName() + " Command Finished\n"));

		if (Constants.Vision.UseLimelight) {
			LimelightHelpers.setPipelineIndex(Constants.Vision.llAprilTag, Constants.Vision.llAprilTagPipelineIndex);
			LimelightHelpers.setPipelineIndex(Constants.Vision.llPython, Constants.Vision.llPythonPipelineIndex);
		}

		if (DriverStation.isFMSAttached()) {
			atComp = true;
			DataLogManager.start(Constants.logDirectory);
		}

		DriverStation.silenceJoystickConnectionWarning(true);
	}

	@Override
	public void robotPeriodic() {
		CommandScheduler.getInstance().run();
		if (Constants.Vision.UseLimelight && Robot.isReal()) {

			var lastResult = LimelightHelpers.getLatestResults(Constants.Vision.llAprilTag).targetingResults;
			Pose2d llPose = lastResult.getBotPose2d_wpiBlue();

			if (LimelightHelpers.getTid("limelight") != -1) {
				Swerve.getInstance().addVisionMeasurement(llPose, Timer.getFPGATimestamp());
			}
		}
		PathPlannerCommand.publishTrajectory(mChooser.getSelected());
	}

	@Override
	public void disabledInit() {}

	@Override
	public void disabledPeriodic() {}

	@Override
	public void disabledExit() {}

	@Override
	public void autonomousInit() {
		new PathPlannerCommand(mChooser.getSelected(), true).schedule();
	}

	@Override
	public void autonomousPeriodic() {}

	@Override
	public void autonomousExit() {
		PathPlannerCommand.unpublishTrajectory();
	}

	@Override
	public void teleopInit() {
		Shooter.getInstance().setNeutralMode(NeutralModeValue.Coast);
	}

	@Override
	public void teleopPeriodic() {}

	@Override
	public void teleopExit() {
		Shooter.getInstance().setNeutralMode(NeutralModeValue.Brake);
	}

	@Override
	public void testInit() {
		CommandScheduler.getInstance().cancelAll();
	}

	@Override
	public void testPeriodic() {}

	@Override
	public void testExit() {}

	@Override
	public void simulationPeriodic() {}

	@Override
	public void simulationInit() {}
}