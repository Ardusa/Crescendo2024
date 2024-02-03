package frc.robot.Commands.Shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Subsystems.Shooter.Shooter;

public class Shoot extends Command {
    private final Shooter shooter = Shooter.getInstance();
    private Timer timer;

    public Shoot() {
        this.setName("Shoot");
        this.addRequirements(shooter);
    }

    @Override
    public void initialize() {
        timer = new Timer();
        timer.start();
    }

    @Override
    public void execute() {
        shooter.shoot(1, 1);
        
        if (timer.get() > 0.5) {
            end(true);
        }
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
