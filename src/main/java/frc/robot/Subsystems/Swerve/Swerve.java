package frc.robot.Subsystems.Swerve;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements
 * subsystem
 * so it can be used in command-based projects easily.
 */
public class Swerve extends SwerveDrivetrain implements Subsystem {
    // private StringPublisher currCmdPub, defCmdPub;
    // private DoubleArrayPublisher posePub;
    private static Swerve mInstance;
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;
    private Field2d mField;

    private final SwerveRequest.ApplyChassisSpeeds autoRequest = new SwerveRequest.ApplyChassisSpeeds();

    public static Swerve getInstance() {
        if (mInstance == null) {
            mInstance = new Swerve(SwerveConstants.DrivetrainConstants, SwerveConstants.FrontLeft,
                    SwerveConstants.FrontRight, SwerveConstants.BackLeft, SwerveConstants.BackRight);
        }
        return mInstance;
    }

    private Swerve(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency,
            SwerveModuleConstants... modules) {
        super(driveTrainConstants, OdometryUpdateFrequency, modules);
        configurePathPlanner();
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    private Swerve(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);
        configurePathPlanner();
        if (Utils.isSimulation()) {
            startSimThread();
        }

        mField = (Field2d) SmartDashboard.getData("Field");
    }

    private void configurePathPlanner() {
        // for (var moduleLocation : m_moduleLocations) {
        // driveBaseRadius = Math.max(driveBaseRadius, moduleLocation.getNorm());
        // }

        AutoBuilder.configureHolonomic(
                () -> this.getState().Pose, // Supplier of current robot pose
                this::seedFieldRelative, // Consumer for seeding pose against auto
                this::getCurrentRobotChassisSpeeds,
                (speeds) -> this.setControl(autoRequest.withSpeeds(speeds)), // Consumer of ChassisSpeeds to drive the
                                                                             // robot
                new HolonomicPathFollowerConfig(
                        Constants.AutoConstants.translationPID,
                        Constants.AutoConstants.rotationPID,
                        SwerveConstants.kSpeedAt12VoltsMetersPerSecond,
                        Constants.SwerveConstants.driveBaseRadius,
                        new ReplanningConfig()),
                this); // Subsystem for requirements
    }

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get())).withName("xDrive");
    }

    public Command getAutoPath(String pathName) {
        return new PathPlannerAuto(pathName);
    }

    public ChassisSpeeds getCurrentRobotChassisSpeeds() {
        return m_kinematics.toChassisSpeeds(getState().ModuleStates);
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    public void addFieldObj(PathPlannerTrajectory trajectory) {
        List<Pose2d> poses = new ArrayList<>();
        // int i = 0;
        AtomicInteger i = new AtomicInteger(0);
        trajectory.getStates().forEach((state) -> {
            if (!(state.getTargetHolonomicPose().equals(trajectory.getInitialTargetHolonomicPose()))
                    && i.get() % 10 == 0)
                poses.add(state.getTargetHolonomicPose());
            i.incrementAndGet();
        });
        // poses.add(trajectory.getInitialTargetHolonomicPose());
        // for (int i = 0; i < trajectory.getStates().size(); i++) {
        // if (i % 10 == 0 && !(trajectory.getState(i).equals(trajectory.getState(i -
        // 1)))) {
        // poses.add(trajectory.getState(i).getTargetHolonomicPose());
        // }
        // }
        // for (State state : trajectory.getStates()) {
        // poses.add(state.poseMeters);
        // }
        mField.getObject(Constants.AutoConstants.kFieldObjectName).setPoses(poses);
    }

    public void addFieldObj(List<Pose2d> poses) {
        mField.getObject(Constants.AutoConstants.kFieldObjectName).setPoses(poses);
    }

    public Field2d getField() {
        return mField;
    }

    public Pose2d getPose() {
        return getState().Pose;
    }

    public int getTID() {
        return (int) NetworkTableInstance.getDefault().getTable("limelight").getEntry("tid").getInteger(0);

    }

    @Override
    public void periodic() {
        try {
            SmartDashboard.putString("Swerve/Default Command", this.getDefaultCommand().getName());
        } catch (NullPointerException e) {
            SmartDashboard.putString("Swerve/Default Command", "empty");
        }
        try {
            SmartDashboard.putString("Swerve/Current Command", this.getCurrentCommand().getName());
        } catch (NullPointerException e) {
            SmartDashboard.putString("Current Command", "empty");
        }

        SmartDashboard.putNumberArray("Swerve/Pose", new double[] {
                getPose().getX(),
                getPose().getY(),
                getPose().getRotation().getDegrees()
        });

        // SmartDashboard.putString("Current Command",
        // this.getCurrentCommand().toString() != null ?
        // this.getCurrentCommand().toString() : "empty");
    }

    // @Override
    // public void initSendable(SendableBuilder builder) {
    // builder.setSmartDashboardType("Subsystem");

    // builder.addBooleanProperty(".hasDefault", () -> getDefaultCommand() != null,
    // null);
    // builder.addStringProperty(
    // ".default",
    // () -> getDefaultCommand() != null ? getDefaultCommand().getName() : "none",
    // null);
    // builder.addBooleanProperty(".hasCommand", () -> getCurrentCommand() != null,
    // null);
    // builder.addStringProperty(
    // ".command",
    // () -> getCurrentCommand() != null ? getCurrentCommand().getName() : "none",
    // null);
    // }
}