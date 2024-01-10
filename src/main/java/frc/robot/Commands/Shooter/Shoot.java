// package frc.robot.Commands.Shooter;

// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.WaitCommand;
// import frc.robot.Constants;
// import frc.robot.Subsystems.Shooter.Arm;
// import frc.robot.Subsystems.Shooter.Shooter;
// import frc.robot.Subsystems.Swerve.Swerve;

// public class Shoot extends Command {
//     private final Arm mArm = Arm.getInstance();
//     private final Shooter mBelt = Shooter.getInstance();
//     private double shootSpeed;

//     public Shoot() {
//         this.setName("Shoot");
//         this.addRequirements(mArm, mBelt);
//         switch (Swerve.getInstance().whatAmILookingAt()) {
//             case kSpeaker:
//                 shootSpeed = Constants.BeltConstants.kBeltSpeedSpeaker;
//                 break;
//             case kAmp:
//                 shootSpeed = Constants.BeltConstants.kBeltSpeedAmp;
//                 break;
//             default: /* Default to speaker */
//                 shootSpeed = Constants.BeltConstants.kBeltSpeedSpeaker;
//                 break;
//         }
//     }

//     @Override
//     public void initialize() {
//         mArm.setArm(Constants.ArmConstants.SetPoints.kShootAngle);
//     }

//     @Override
//     public void execute() {
//         mArm.aim();
//     }

//     @Override
//     public void end(boolean interrupted) {
//         mBelt.shoot(shootSpeed);
//         new WaitCommand(0.5).schedule();
//         mBelt.stop();
//     }
// }
