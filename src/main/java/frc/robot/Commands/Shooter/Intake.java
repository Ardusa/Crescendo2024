package frc.robot.Commands.Shooter;

import com.ctre.phoenix6.StatusSignal;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
// import frc.robot.Subsystems.Shooter.Arm;
import frc.robot.Subsystems.Shooter.Shooter;
import java.util.function.*;

public class Intake extends Command {
    // private final Arm mArm = Arm.getInstance();
    private final Shooter mBelt = Shooter.getInstance();
    private Supplier<Double> thingy, thingy2;

    public Intake(Supplier<Double> thing, Supplier<Double> thing2) {
        this.setName("Intake");
        // this.addRequirements(mArm, mBelt);
        this.addRequirements(mBelt);
        thingy = thing;
        thingy2 = thing2;
    }

    @Override
    public void initialize() {
        // mArm.setArm(Constants.ArmConstants.SetPoints.kIntakeAngle);
    }

    @Override
    public void execute() {
        mBelt.shoot(thingy.get(), thingy2.get());
    }

    @Override
    public void end(boolean interrupted) {
        // mArm.setArm(Constants.ArmConstants.SetPoints.kShootAngle);
    }

    @Override
    public boolean isFinished() {
        return mBelt.getHolding();
    }
}
