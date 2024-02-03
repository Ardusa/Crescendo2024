// package frc.robot.Commands;

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Subsystems.Intake;

// public class HoldIntake extends Command {
//     private Intake mIntake;

//     public HoldIntake() {
//         mIntake = Intake.getInstance();
//         this.addRequirements(mIntake);
//         this.setName("Intake");
//     }

//     @Override
//     public void initialize() {
//         mIntake.setHolding(false);
//     }

//     @Override
//     public void execute() {
//         mIntake.setDeployed(true);
//     }

//     @Override
//     public void end(boolean interrupted) {
//         mIntake.setDeployed(false);
//         mIntake.setHolding(true);
//         if (interrupted) {
//             mIntake.setHolding(false);
//         }
//     }

//     @Override
//     public boolean isFinished() {
//         return mIntake.getHolding();
//     }

// }
