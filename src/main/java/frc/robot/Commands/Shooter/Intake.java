// package frc.robot.Commands.Shooter;

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Constants;
// import frc.robot.Subsystems.Shooter.Arm;
// import frc.robot.Subsystems.Shooter.Shooter;

// public class Intake extends Command {
//     private final Arm mArm = Arm.getInstance();
//     private final Shooter mBelt = Shooter.getInstance();

//     public Intake() {
//         this.setName("Intake");
//         this.addRequirements(mArm, mBelt);
//     }

//     @Override
//     public void initialize() {
//         mArm.setArm(Constants.ArmConstants.SetPoints.kIntakeAngle);
//     }

//     @Override
//     public void execute() {
//         mBelt.intake();
//     }

//     @Override
//     public void end(boolean interrupted) {
//         mArm.setArm(Constants.ArmConstants.SetPoints.kShootAngle);
//     }

//     @Override
//     public boolean isFinished() {
//         return mBelt.getHolding();
//     }
// }
