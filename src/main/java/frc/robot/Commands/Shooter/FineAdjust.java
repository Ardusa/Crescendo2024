// package frc.robot.Commands.Shooter;

// import java.util.function.Supplier;

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Subsystems.Shooter.Arm;

// public class FineAdjust extends Command {
//     private final Arm arm = Arm.getInstance();
//     private Supplier<Double> manualAdjust;


//     public FineAdjust(Supplier<Double> manualAdjust) {
//         this.manualAdjust = manualAdjust;
//         this.setName("FineAdjust");
//         this.addRequirements(arm);
//     }

//     // Called every time the scheduler runs while the command is scheduled.
//     @Override
//     public void execute() {
//         arm.setArmVelocity(manualAdjust.get());
//     }

//     // Returns true when the command should end.
//     @Override
//     public boolean isFinished() {
//         return false;
//     }
// }
