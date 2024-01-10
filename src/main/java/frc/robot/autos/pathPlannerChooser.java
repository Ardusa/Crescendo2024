package frc.robot.autos;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Swerve.Swerve;

public class pathPlannerChooser {
    public enum PathPlannerType {
        AUTO,
        PATH
    }
    private final Swerve mSwerve;
    public boolean autonFinished;
    // private PathPlannerTrajectory pathPlannerTrajectory;
    private List<PathPlannerPath> paths = new ArrayList<>();
    // private PathPlannerAuto pathPlannerAuto;
    private String commandName = "";

    // private HashMap<String, Command> eventMap1 = new HashMap<String, Command>();

    /** Do not include file extensions: .auto, .json, .path */
    public pathPlannerChooser(String str) {
        mSwerve = Swerve.getInstance();
        autonFinished = false;

        commandName = str;

        // if (!str.equals("null")) {
        //     try {
        //         switch (type) {
        //             case AUTO:                        
        //             paths = PathPlannerAuto.getPathGroupFromAutoFile(str);
        //                 break;
        //             case PATH:
        //                 paths.add(PathPlannerPath.fromPathFile(str));
        //                 break;
        //             default:
        //                 break;
        //         }
        //         pathPlannerTrajectory = path.getTrajectory(mSwerve.getCurrentRobotChassisSpeeds(),
        //                 mSwerve.getState().Pose.getRotation());
        //     } catch (Exception e) {
        //         DriverStation.reportError("PathPlannerChooser: Path not found: " + str, false);
        //         /* Blank path that will just chill */
        //         List<Translation2d> blank = new ArrayList<>();
        //         blank.add(mSwerve.getState().Pose.getTranslation());
        //         blank.add(mSwerve.getState().Pose.getTranslation());
        //         System.out.println("PathPlannerChooser: Path not found: " + str);
        //         paths.add(new PathPlannerPath(blank, new PathConstraints(0, 0, 0, 0),
        //                 new GoalEndState(0, new Rotation2d(0), false)));
        //         commandName = e.toString();
        //     }

            // mSwerve.seedFieldRelative(paths.get(0).getPreviewStartingHolonomicPose());

            List<Pose2d> poses = new ArrayList<>();
            for (PathPlannerPath path : paths) {
                for (PathPoint point : path.getAllPathPoints()) {
                    poses.add(new Pose2d(point.position, point.rotationTarget.getTarget()));
                }
            }
            autonPoses(poses);
            try {
                mSwerve.seedFieldRelative(paths.get(0).getPreviewStartingHolonomicPose());
            } catch (IndexOutOfBoundsException e) {
                System.out.println("PathPlannerChooser: Path not found: " + paths);
            }
        // }
        // eventMap1.put("done", new InstantCommand(() -> autonFinished = true));
        // eventMap1.put("start", Commands.print("Start"));
        // eventMap1.put("print", Commands.print("print"));
        // eventMap1.put("testmark", new InstantCommand(() -> System.out.println("Test Mark")));
    }

    public Command generateTrajectory() {
        return AutoBuilder.buildAuto(commandName);
        // if (commandName.equals("null")) {
        //     return Commands.print("Null Path");
        // } else {
        //     AutoBuilder.configureHolonomic(
        //             () -> mSwerve.getState().Pose,
        //             (pose) -> mSwerve.seedFieldRelative(pose),
        //             () -> mSwerve.getCurrentRobotChassisSpeeds(),
        //             (speeds) -> mSwerve.applyRequest(() -> new SwerveRequest.ApplyChassisSpeeds().withSpeeds(speeds)),
        //             new HolonomicPathFollowerConfig(SwerveConstants.kSpeedAt12VoltsMps,
        //                     SwerveConstants.driveBaseRadius,
        //                     new ReplanningConfig(true, true)),
        //             mSwerve);
        //     return AutoBuilder.followPathWithEvents(path);
        // }
    }

    // public PathPlannerPath getPath() {
    //     return path;
    // }

    private void autonPoses(List<Pose2d> poses) {
        mSwerve.addFieldObj(poses);
    }
}
