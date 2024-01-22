package frc.robot.autos;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Subsystems.Swerve.Swerve;

public class pathPlannerCommand extends SequentialCommandGroup {
    private Swerve swerve = Swerve.getInstance();
    // private Shooter shooter = Shooter.getInstance();

    public pathPlannerCommand(String autoName) {
        // Pose2d startingPose = PathPlannerAuto.getPathGroupFromAutoFile(autoName).get(0)
        //         .getPreviewStartingHolonomicPose();
        // Swerve.getInstance().seedFieldRelative(startingPose, startingPose.getRotation().rotateBy(new Rotation2d(Math.PI)));
        swerve.seedFieldRelative(PathPlannerAuto.getStaringPoseFromAutoFile(autoName));

        // /* TODO: Add all named commands in pathplanner app */
        // NamedCommands.registerCommand("Shoot", new Shoot());
        // NamedCommands.registerCommand("Intake",
        //         new SequentialCommandGroup(
        //                 new SetPoint(Constants.ArmConstants.SetPoints.kIntakeAngle),
        //                 new ParallelDeadlineGroup(new WaitCommand(1), new Intake()),
        //                 new SetPoint(Constants.ArmConstants.SetPoints.kShootAngle)
        //     )
        // );

        // if (shootFirst) {
        //     this.addCommands(new Shoot(), // Shoot
        //     new SetPoint(Constants.ArmConstants.SetPoints.kIntakeAngle)); // Deploy intake for pickup
        // } else {
        //     this.addCommands(new SetPoint(Constants.ArmConstants.SetPoints.kIntakeAngle)); // Deploy intake for pickup immediately
        // }

        this.setName(autoName);
        this.addRequirements(swerve);

        this.addCommands(
                AutoBuilder.buildAuto(autoName).withName(autoName));
    }
}
