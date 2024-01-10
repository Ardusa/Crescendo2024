// package frc.robot.Commands.Shooter;

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Constants;
// import frc.robot.Subsystems.Shooter.Shooter;

// public class BeltDrive extends Command {
//     private final Shooter mBelt = Shooter.getInstance();

//     public BeltDrive() {
//         this.setName("Belt Drive");
//         this.addRequirements(mBelt);
//     }

//     @Override
//     public void execute() {
//         mBelt.intakeSpeed(Constants.BeltConstants.kBeltFeedForward);
//     }

//     @Override
//     public boolean isFinished() {
//         return false;
//     }
// }
