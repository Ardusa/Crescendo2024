package frc.robot.Commands.Shooter;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Subsystems.Shooter.Arm;
import frc.robot.Subsystems.Shooter.Shooter;
import frc.robot.Subsystems.Swerve.Swerve;

public class AimAndShoot extends SequentialCommandGroup {
    private Swerve mSwerve;
    private Arm mArm;
    private Shooter mShooter;
    private Constants.SwerveConstants.Target target;
    private double setpoint;

    public AimAndShoot() {
        mSwerve = Swerve.getInstance();
        mArm = Arm.getInstance();
        mShooter = Shooter.getInstance();

        this.setName("Aim and Shoot");
        this.addRequirements(mSwerve, mArm, mShooter);

        this.addCommands(new InstantCommand(() -> {
            target = mSwerve.whatAmILookingAt();
            if (target.equals(Constants.SwerveConstants.Target.kAmp)) {
                setpoint = Constants.ArmConstants.SetPoints.kAmp;
            } else {
                setpoint = mArm.aim();
            }
        }),

                new SetPoint(setpoint).withTimeout(1)
                        .until(() -> mArm.isInRangeOfTarget(Constants.ArmConstants.SetPoints.kIntake)));
    }
}