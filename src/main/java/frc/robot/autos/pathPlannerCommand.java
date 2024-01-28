package frc.robot.autos;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Commands.Swerve.Align;
import frc.robot.Subsystems.Swerve.Swerve;

public class PathPlannerCommand extends SequentialCommandGroup {
    private Swerve swerve;
    private boolean nullAuto;

    public PathPlannerCommand(String autoName, boolean shoot) {
        swerve = Swerve.getInstance();
        nullAuto = false;
        try {
            swerve.seedFieldRelative(new Pose2d(PathPlannerAuto.getStaringPoseFromAutoFile(autoName).getTranslation(),
                    Rotation2d.fromDegrees(0)));
        } catch (Exception e) {
            nullAuto = true;
            this.setName("Null Auto");
            System.out.println("Null auto");
        }

        this.addCommands(
                new Align(() -> 0, () -> 0, Constants.SwerveConstants.kMaxSpeedMetersPerSecond)
                        .until(() -> new WaitCommand(1).isFinished())
                // Next it must shoot
        );

        if (!nullAuto) {
            this.setName(autoName);
            this.addRequirements(swerve);
            this.addCommands(
                    AutoBuilder.buildAuto(autoName));
        }
    }

    public static void publishTrajectory(String autoName) {
        if (autoName.equals("null")) {
            Robot.mField.getObject(Constants.AutoConstants.kFieldObjectName)
                    .setPose(new Pose2d(-5, -5, Rotation2d.fromDegrees(0)));
            return;
        }

        List<Pose2d> poses = new ArrayList<>();
        poses.clear();

        PathPlannerAuto.getPathGroupFromAutoFile(autoName).forEach(
                (path) -> path.getAllPathPoints().forEach(
                        (point) -> poses.add(new Pose2d(point.position, point.position.getAngle()))));

        Swerve.getInstance().addFieldObj(poses);
    }

    public static void unpublishTrajectory() {
        Swerve.getInstance().getField().close();
    }
}
