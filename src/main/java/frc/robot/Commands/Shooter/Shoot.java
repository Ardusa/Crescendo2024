package frc.robot.Commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Subsystems.Shooter.Shooter;

public class Shoot extends Command {
    private final Shooter shooter = Shooter.getInstance();
    private double shootSpeed;

    public Shoot() {
        this.setName("Shoot");
        this.addRequirements(shooter);
        shootSpeed = Constants.BeltConstants.kBeltSpeedSpeaker;
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        shooter.shoot(01, 01);
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            shooter.stop();
            shooter.toggleHolding();
        }
    }

    @Override
    public boolean isFinished() {
        return !shooter.getHolding();
    }
}
