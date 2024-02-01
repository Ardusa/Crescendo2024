package frc.robot.Commands.Shooter;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Subsystems.Shooter.Shooter;

public class Shoot extends Command {
    private final Shooter shooter = Shooter.getInstance();
    private double shootSpeed;
    private DoubleSupplier input1, input2;

    /**
     * 
     * @param input1 shooter
     * @param input2 feeder
     */
    public Shoot(DoubleSupplier input1, DoubleSupplier input2) {
        this.input1 = input1;
        this.input2 = input2;
        this.setName("Shoot");
        this.addRequirements(shooter);
        shootSpeed = Constants.BeltConstants.kBeltSpeedSpeaker;
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        shooter.shoot(input1.getAsDouble(), input2.getAsDouble());
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
