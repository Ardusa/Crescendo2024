// package frc.robot.Commands.Shooter;

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Constants;
// import frc.robot.Utils;
// // import frc.robot.Subsystems.Shooter.Arm;
// import frc.robot.Subsystems.Shooter.Shooter;
// import frc.robot.Subsystems.Swerve.Swerve;

// public class Shoot extends Command {
//     // private final Arm mArm = Arm.getInstance();
//     private final Shooter mBelt = Shooter.getInstance();
//     private double shootSpeed;

//     public Shoot() {
//         this.setName("Shoot");
//         // this.addRequirements(mArm, mBelt);
//         this.addRequirements(mBelt);
//         if (Utils.withinRange(Swerve.getInstance().getState().Pose.getRotation().getDegrees(), 20)) {
//             shootSpeed = Constants.BeltConstants.kBeltSpeedAmp;
//         } else {
//             shootSpeed = Constants.BeltConstants.kBeltSpeedSpeaker;
//         }
//     }

//     @Override
//     public void initialize() {
//         // mArm.setArm(Constants.ArmConstants.SetPoints.kShootAngle);
//     }

//     @Override
//     public void execute() {
//         mBelt.shoot(shootSpeed);
//     }

//     @Override
//     public void end(boolean interrupted) {
//         mBelt.stop();
//     }

//     @Override
//     public boolean isFinished() {
//         return !mBelt.getHolding();
//     }
// }
