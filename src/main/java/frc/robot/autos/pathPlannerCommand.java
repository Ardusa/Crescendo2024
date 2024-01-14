// package frc.robot.autos;

// import com.pathplanner.lib.auto.AutoBuilder;
// import com.pathplanner.lib.auto.NamedCommands;

// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import frc.robot.Constants;
// import frc.robot.Subsystems.Swerve.Swerve;

// public class pathPlannerCommand extends SequentialCommandGroup {
//     private Swerve mSwerve = Swerve.getInstance();

//     public pathPlannerCommand(String autoName, boolean shootFirst) {
//         /* TODO: Add all named commands in pathplanner app */
//         NamedCommands.registerCommand("Shoot", new Shoot());
//         NamedCommands.registerCommand("Intake",
//                 new SequentialCommandGroup(
//                         new deployIntake(), // Deploy intake for pickup
//                         new Intake(), // Intake
//                         new retractIntake() // Retract intake
//             )
//         );

//         if (shootFirst) {
//             this.addCommands(new Shoot().withTimeout(0.5), // Shoot
//             new deployIntake()); // Deploy intake for pickup
//         } else {
//             this.addCommands(new deployIntake()); // Deploy intake for pickup immediately
//         }

//         this.setName(autoName);
//         this.addRequirements(mSwerve);

//         this.addCommands(
//                 AutoBuilder.buildAuto(autoName));
//     }
// }
