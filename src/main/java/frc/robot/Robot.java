// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Subsystems.Swerve.Swerve;
import frc.robot.autos.pathPlannerCommand;

public class Robot extends TimedRobot {
	private Command mAutonomousCommand;
	private SendableChooser<String> mChooser = new SendableChooser<>();
	public static Field2d mField = new Field2d();

	@Override
	public void robotInit() {
		DataLogManager.start("WPILog", "", 1);
		SmartDashboard.putData("Field", mField);
		new RobotContainer();
		DriverStation.silenceJoystickConnectionWarning(true);

		mChooser.setDefaultOption("Do Nothing", "null");
		AutoBuilder.getAllAutoNames().forEach((name) -> {
			mChooser.addOption(name, name);
		});
		SmartDashboard.putData("Auton Chooser", mChooser);

		Swerve.getInstance().getDaqThread().setThreadPriority(99);

		CommandScheduler.getInstance()
				.onCommandInitialize((action) -> DataLogManager.log("Intializing " + action.getName()));
		CommandScheduler.getInstance()
				.onCommandInterrupt((action) -> DataLogManager.log("Interrupting " + action.getName()));
		CommandScheduler.getInstance()
				.onCommandFinish((action) -> DataLogManager.log("Finished " + action.getName()));
	}

	@Override
	public void robotPeriodic() {
		CommandScheduler.getInstance().run();
		if (Constants.UseLimelight) {
			
			var lastResult = LimelightHelpers.getLatestResults("limelight").targetingResults;
			Pose2d llPose = lastResult.getBotPose2d_wpiBlue();

			if (lastResult.valid) {
				Swerve.getInstance().addVisionMeasurement(llPose, Timer.getFPGATimestamp());
			}
		}
	}

	@Override
	public void disabledInit() {
	}

	@Override
	public void disabledPeriodic() {
	}

	@Override
	public void disabledExit() {
	}

	@Override
	public void autonomousInit() {
		mAutonomousCommand = new pathPlannerCommand(mChooser.getSelected(), true);
		// mAutonomousCommand = new
		// pathPlannerChooser(mChooser.getSelected()).generateTrajectory();

		if (mAutonomousCommand != null) {
			mAutonomousCommand.schedule();
		}
	}

	@Override
	public void autonomousPeriodic() {
	}

	@Override
	public void autonomousExit() {
	}

	@Override
	public void teleopInit() {
		if (mAutonomousCommand != null) {
			mAutonomousCommand.cancel();
		}
	}

	@Override
	public void teleopPeriodic() {
	}

	@Override
	public void teleopExit() {
	}

	@Override
	public void testInit() {
		CommandScheduler.getInstance().cancelAll();
	}

	@Override
	public void testPeriodic() {
	}

	@Override
	public void testExit() {
	}

	@Override
	public void simulationPeriodic() {
	}

	@Override
	public void simulationInit() {
	}
}