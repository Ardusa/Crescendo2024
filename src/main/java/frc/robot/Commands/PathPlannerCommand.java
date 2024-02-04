package frc.robot.Commands;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.AutoBuilderException;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Subsystems.Shooter.Arm;
import frc.robot.Subsystems.Swerve.Swerve;

public class PathPlannerCommand extends Command {
    private Swerve swerve;
    private Arm arm;
    private Command auto;
    private Pose2d startPose;
    private SequentialCommandGroup autoGroup;

    private static String lastAutoName;

    public PathPlannerCommand(String autoName, boolean shoot) {
        NamedCommands.registerCommand("Shoot", new PrintCommand("Shoot Command, PathPlanner"));
        NamedCommands.registerCommand("Intake", new PrintCommand("Intake Command, PathPlanner"));

        swerve = Swerve.getInstance();
        arm = Arm.getInstance();

        try {
            auto = AutoBuilder.buildAuto(autoName);
        } catch (AutoBuilderException e) {
            this.setName("Null Auto");
            System.out.println("Null auto");
            auto = new PrintCommand("Null Auto");
        }
        this.setName(autoName);
        this.addRequirements(swerve);
        startPose = PathPlannerAuto.getPathGroupFromAutoFile(autoName).get(0).getPreviewStartingHolonomicPose();
    }

    @Override
    public void initialize() {
        swerve.seedFieldRelative(startPose);
        System.out.println("Starting Pose: " + startPose.toString());
        // autoGroup = new SequentialCommandGroup(new HoldToPosition(Constants.ArmConstants.SetPoints.kAmp),
                // new Shoot().unless(() -> new WaitCommand(0.5).isFinished()),
                // auto);
        // SmartDashboard.putData("Auto", autoGroup);
        // autoGroup.schedule();
        auto.schedule();
    }
    

    @Override
    public void execute() {}

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
        if (autoName.equals(lastAutoName)) {
            return;
        } else if (autoName.equals("null")) {
            Robot.mField.getObject(Constants.AutoConstants.kFieldObjectName)
                    .setPose(new Pose2d(-5, -5, Rotation2d.fromDegrees(0)));
            return;
        } else {
            lastAutoName = autoName;
        }

        List<Pose2d> poses = new ArrayList<>();
        poses.clear();

        PathPlannerAuto.getPathGroupFromAutoFile(autoName).forEach((path) -> path.getAllPathPoints()
                .forEach((point) -> poses.add(new Pose2d(point.position, point.position.getAngle()))));

        Swerve.getInstance().addFieldObj(poses);
    }

    public static void unpublishTrajectory() {
        Swerve.getInstance().getField().getObject(Constants.AutoConstants.kFieldObjectName)
                .setPose(new Pose2d(-5, -5, Rotation2d.fromDegrees(0)));
    }
}