package frc.robot.autos;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Commands.Shooter.Shoot;
import frc.robot.Subsystems.Swerve.Swerve;

public class PathPlannerCommand extends Command {
    private Swerve swerve;
    private Command auto;

    public PathPlannerCommand(String autoName, boolean shoot) {
        NamedCommands.registerCommand("Shoot", new PrintCommand("Shoot Command, PathPlanner"));
        NamedCommands.registerCommand("Intake", new PrintCommand("Intake Command, PathPlanner"));

        // if (Robot.isReal()) {
        //     startingRotation = Rotation2d.fromDegrees(180);
        // } else {
        //     startingRotation = Rotation2d.fromDegrees(270);
        // }

        swerve = Swerve.getInstance();
        try {
            auto = AutoBuilder.buildAuto(autoName);
        } catch (RuntimeException e) {
            this.setName("Null Auto");
            System.out.println("Null auto");
            auto = new PrintCommand("Null Auto");
        }
            this.setName(autoName);
            this.addRequirements(swerve);
    }

    @Override
    public void initialize() {
        new Shoot().schedule();
        Pose2d startPose = new Pose2d(2, 4, Rotation2d.fromDegrees(157.5));
        swerve.seedFieldRelative(startPose);
        System.out.println("Starting Pose: " + startPose.toString());
        auto.schedule();
    }

    @Override
    public void execute() {
    }
    
    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            auto.cancel();
        }
    }

    @Override
    public boolean isFinished() {
        return auto.isFinished();
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
        Swerve.getInstance().getField().getObject(Constants.AutoConstants.kFieldObjectName)
                .setPose(new Pose2d(-5, -5, Rotation2d.fromDegrees(0)));
    }
}