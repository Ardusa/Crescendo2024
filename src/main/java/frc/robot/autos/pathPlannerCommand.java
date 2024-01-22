package frc.robot.autos;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Subsystems.Swerve.Swerve;

public class pathPlannerCommand extends SequentialCommandGroup {
    private Swerve swerve = Swerve.getInstance();

    public pathPlannerCommand(String autoName) {
        swerve.seedFieldRelative(PathPlannerAuto.getStaringPoseFromAutoFile(autoName));

        this.setName(autoName);
        this.addRequirements(swerve);

        this.addCommands(
                AutoBuilder.buildAuto(autoName).withName(autoName));
    }
}
