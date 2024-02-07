// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Map;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Commands.PathPlannerCommand;
import frc.robot.Subsystems.Shooter.Shooter;
import frc.robot.Subsystems.Swerve.Swerve;

public class Robot extends TimedRobot {
	public static Field2d mField = new Field2d();

	private SendableChooser<String> mChooser = new SendableChooser<>();
	private Timer timer;
	private Command auton;
	private ShuffleboardTab autonTab;
	private NetworkTable autonTable;
	private NetworkTableEntry closePieceCount, farPieceCount, shoot, startPosition;

	public static boolean atComp = false;
	public static boolean autonomousExited = false;

	/* TODO: Set this value for every auto */
	public static final boolean shootFirst = true;

	@Override
	public void robotInit() {
		timer = new Timer();
		DataLogManager.start();

		new RobotContainer();

		autonTable = NetworkTableInstance.getDefault().getTable("Autonomous");
		autonTab = Shuffleboard.getTab("Autonomous");
		SendableChooser<String> startPosition = new SendableChooser<>();

		autonTab.add("How many close pieces", 0).withWidget(BuiltInWidgets.kNumberSlider)
				.withProperties(Map.of("min", 0, "max", 3, "block increment", 1)).withSize(2, 1);

		autonTab.add("How many far pieces", 0).withWidget(BuiltInWidgets.kNumberSlider)
				.withProperties(Map.of("min", 0, "max", 3, "block increment", 1)).withSize(2, 1);

		autonTab.add("Shoot", true).withWidget(BuiltInWidgets.kToggleButton).withSize(1, 1);

		startPosition.setDefaultOption("CHANGE NOW DEFAULTS TO LEFT", "left");
		startPosition.addOption("Left", "left");
		startPosition.addOption("Right", "right");
		startPosition.addOption("Center", "center");
		autonTab.add("Start Position", startPosition).withWidget(BuiltInWidgets.kComboBoxChooser).withSize(2, 1);

		this.closePieceCount = NetworkTableInstance.getDefault().getTable("Shuffleboard").getSubTable("Autonomous")
				.getEntry("How many close pieces");
		this.farPieceCount = NetworkTableInstance.getDefault().getTable("Shuffleboard").getSubTable("Autonomous")
				.getEntry("How many far pieces");
		this.shoot = NetworkTableInstance.getDefault().getTable("Shuffleboard").getSubTable("Autonomous")
				.getEntry("Shoot");
		this.startPosition = NetworkTableInstance.getDefault().getTable("Shuffleboard").getSubTable("Autonomous")
				.getSubTable("Start Position").getEntry("active");
				
		autonTable = NetworkTableInstance.getDefault().getTable("Shuffleboard").getSubTable("Autonomous");

		/* All auton information */

		SmartDashboard.putData("Field", mField);
		autonTab.add("Field", mField).withWidget(BuiltInWidgets.kField);


		// mChooser.setDefaultOption("Do Nothing", "null");
		// AutoBuilder.getAllAutoNames().forEach((name) -> mChooser.addOption(name, name));
		// SmartDashboard.putData("Auton Chooser", mChooser);



		// autonTable.getEntry("Shoot").setBoolean(true);
		// // NetworkTableInstance.getDefault().getTable("Autonomous").getEntry("Shoot First").setBoolean(true);

		Swerve.getInstance().getDaqThread().setThreadPriority(99);

		CommandScheduler.getInstance()
				.onCommandInitialize((action) -> DataLogManager.log(action.getName() + " Command Initialized"));
		CommandScheduler.getInstance()
				.onCommandInterrupt((action) -> DataLogManager.log(action.getName() + " Command Interrupted"));
		CommandScheduler.getInstance()
				.onCommandFinish((action) -> DataLogManager.log(action.getName() + " Command Finished"));

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

	@SuppressWarnings("unused")

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
		if (!autonomousExited) {
			// PathPlannerCommand.publishTrajectory(mChooser.getSelected());
		}
	}

	@Override
	public void disabledInit() {}

	@Override
	public void disabledPeriodic() {}

	@Override
	public void disabledExit() {}

	@Override
	public void autonomousInit() {
		timer = new Timer();
		auton = new PathPlannerCommand(startPosition.getString("left"), closePieceCount.getInteger(0),
				farPieceCount.getInteger(0), false);

		// auton = new PathPlannerCommand(mChooser.getSelected(), this.shoot.getBoolean(false));
		auton.schedule();
		timer.restart();
	}

	@Override
	public void autonomousPeriodic() {
		NetworkTableInstance.getDefault().getTable("PathPlanner").getEntry("Auto Timer").setDouble(timer.get());
		if (!CommandScheduler.getInstance().isScheduled(auton)) {
			timer.stop();
		}
	}

	@Override
	public void autonomousExit() {
		if (atComp) {
			PathPlannerCommand.unpublishTrajectory();
			autonomousExited = true;
		}
	}

	@Override
	public void teleopInit() {
		Shooter.getInstance().setBrakeMode(false);
	}

	@Override
	public void teleopPeriodic() {}

	@Override
	public void teleopExit() {
		Shooter.getInstance().setBrakeMode(true);
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