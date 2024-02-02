package frc.robot.Commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Shooter.Arm;

public class SetPoint extends Command {
    private final Arm arm = Arm.getInstance();
    private double position;

    /**
     * Set the arm to a specific position
     * @param position in degrees
     */
    public SetPoint(double position) {
        this.position = position;
        this.setName("SetPoint");
        this.addRequirements(arm);
    }

    @Override
    public void initialize() {
        arm.setArmToPosition(position);
    }

    // @Override
    // public void execute() {
    //     System.out.println(arm.getArmRotation());
    // }

    @Override
    public boolean isFinished() {
        return arm.isInRangeOfTarget(position);
    }
}
