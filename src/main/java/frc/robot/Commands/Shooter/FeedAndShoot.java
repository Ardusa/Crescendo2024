package frc.robot.Commands.Shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Shooter.Shooter;

public class FeedAndShoot extends Command {
    private final Shooter shooter;
    private Timer timer;

    public FeedAndShoot() {
        timer = new Timer();
        shooter = Shooter.getInstance();
        this.setName("Shoot");
        this.addRequirements(shooter);
    }

    @Override
    public void initialize() {
        timer.restart();
    }

    @Override
    public void execute() {
        if (timer.get() < 0.2) {
            shooter.shoot(0, 0.3);
        } else {
            shooter.shoot(01, 0);
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop();

        if (!(timer.get() < 0.3)) {
            shooter.toggleHolding();
        }
    }

    @Override
    public boolean isFinished() {
        return timer.get() > 0.5;
    }
}
