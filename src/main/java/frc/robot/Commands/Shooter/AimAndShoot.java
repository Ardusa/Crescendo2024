package frc.robot.Commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Shooter.Arm;

public class AimAndShoot extends Command {
    private Arm mArm;
    private double setpoint;

    public AimAndShoot() {
        mArm = Arm.getInstance();

        this.setName("Aim and Shoot");
    }

    @Override
    public void initialize() {
        // TODO: not aiming correctly, but printing the correct setpoint
        setpoint = mArm.calculateArmSetpoint();
        System.out.println("AimAndShoot: " + setpoint);
        new SetPoint(setpoint).withTimeout(0.5).andThen(new FeedAndShoot().withTimeout(0.5));
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}