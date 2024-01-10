// package frc.robot.autos;

// import java.io.IOException;
// import java.nio.file.Path;
// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.controller.ProfiledPIDController;
// import edu.wpi.first.math.trajectory.Trajectory;
// import edu.wpi.first.math.trajectory.TrajectoryUtil;
// import edu.wpi.first.wpilibj.DataLogManager;
// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.Filesystem;
// import edu.wpi.first.wpilibj2.command.InstantCommand;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
// import frc.robot.Constants;
// import frc.robot.Robot;
// import frc.robot.commands.Swerve.balance;
// import frc.robot.subsystems.Swerve;

// public class autoFromPath extends SequentialCommandGroup {
//     public autoFromPath() {
//         Swerve mSwerve = Swerve.getInstance();
//         addRequirements(mSwerve);
//         String path = "paths/" + Robot.chooser.getSelected();
//         DataLogManager.log(path);

//         // An example trajectory to follow. All units in meters.

//         try {
//             Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(path);
//             Trajectory trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
//             var thetaController = new ProfiledPIDController(
//                     Constants.AutoConstants.kPThetaController, 0, 0,
//                     Constants.AutoConstants.kThetaControllerConstraints);
//             thetaController.enableContinuousInput(-Math.PI, Math.PI);

//             // if(SmartDashboard.getBoolean("isRed", false)) {
//             // mSwerve.setGyro(0);
//             mSwerve.updateOdometryManual(trajectory.getInitialPose().getX(),
//                     trajectory.getInitialPose().getY(),
//                     trajectory.getInitialPose().getRotation().getDegrees());

//             mSwerve.addFieldObj(trajectory);

//             SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
//                     trajectory,
//                     mSwerve::getPose,
//                     Constants.Swerve.swerveKinematics,
//                     new PIDController(Constants.AutoConstants.kPXController, 0, 0),
//                     new PIDController(Constants.AutoConstants.kPYController, 0, 0),
//                     thetaController,
//                     mSwerve::setModuleStates,
//                     mSwerve);

//             addCommands(
//                     new InstantCommand(() -> mSwerve.lockPosition()),
//                     swerveControllerCommand,
//                     new balance());
//         } catch (IOException ex) {

//             DriverStation.reportError("Unable to Open Trajectory" + Filesystem.getDeployDirectory(),
//                     ex.getStackTrace());
//         }
//     }
// }
