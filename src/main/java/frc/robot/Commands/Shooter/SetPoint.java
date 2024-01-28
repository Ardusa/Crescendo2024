package frc.robot.Commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Shooter.Arm;
import frc.robot.Utils;

public class SetPoint extends Command {
    private final Arm arm = Arm.getInstance();
    private double position;


    public SetPoint(double position) {
        this.position = position;
        this.setName("SetPoint");
        this.addRequirements(arm);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        arm.setArm(arm.inRange(position));
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        System.out.println(arm.getArmRotation());
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return Utils.withinRange(arm.getArmRotation(), 15);
    }
}
