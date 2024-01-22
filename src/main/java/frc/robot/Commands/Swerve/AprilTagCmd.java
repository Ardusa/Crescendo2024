// package frc.robot.Commands.Swerve;

// import com.pathplanner.lib.util.PIDConstants;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.networktables.NetworkTable;
// import edu.wpi.first.networktables.NetworkTableEntry;
// import edu.wpi.first.networktables.NetworkTableInstance;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.PIDCommand;
// import frc.robot.Constants;
// import frc.robot.LimelightHelpers;
// import frc.robot.Utils;
// import frc.robot.Subsystems.Swerve.Swerve;
// import frc.robot.Subsystems.Swerve.SwerveRequest;

// public class AprilTagCmd extends Command {
//     private Swerve mSwerve = Swerve.getInstance();
//     private int targetID;
//     private Constants.AprilTag target;
//     private SwerveRequest.RobotCentric forwardStraight;
//     // private PIDConstants pid;
//     private PIDController xController, yController;
//     // private PIDCommand command;


//     public AprilTagCmd() {
//         this.setName("AprilTagCmd");
//     }

//     @Override
//     public void initialize() {
//         // xController = new PIDController(0.2, 0, 0);
//         yController = new PIDController(0.2, 0, 0);
//         yController.setSetpoint(0);
//         yController.setTolerance(10);

//         forwardStraight = new SwerveRequest.RobotCentric();
//         // pid = new PIDConstants(1, 1, 1);
//     }
    
//     @Override
//     public void execute() {
//         // targetID = (int) NetworkTableInstance.getDefault().getTable("limelight").getEntry("tid").getInteger(0);
//         target = Constants.AprilTag.fromId(mSwerve.getTID());
//         switch (target) {
//             case BlueAmp:
//                 System.out.println("Blue Amp");
//                 break;
//             case BlueCenterStage:
//                 System.out.println("Blue Center Stage");
//                 break;
//             case BlueLeftHumanPlayer:
//                 System.out.println("Blue Left Human Player");
//                 break;
//             case BlueLeftStage:
//                 System.out.println("Blue Left Stage");
//                 break;
//             case BlueRightHumanPlayer:
//                 System.out.println("Blue Right Human Player");
//                 break;
//             case BlueRightStage:
//                 System.out.println("Blue Right Stage");
//                 break;
//             case BlueSpeaker:
//                 // System.out.println("Blue Speaker");
                
//                 mSwerve.applyRequest(() -> forwardStraight.withVelocityX(0)
//                         .withVelocityY(yController.calculate(LimelightHelpers.getTX("limelight"))));

//                 System.out.println(yController.getPositionError());
//                 // mSwerve.applyRequest(() -> forwardStraight.withVelocityX(xController.calculate(0)).withVelocityY(yController.calculate(0, LimelightHelpers.getTX("limelight"))));
//                 break;
//             case BlueSpeakerOffset:
//                 System.out.println("Blue Speaker Offset");
//                 break;
//             case NoTag:
//                 System.out.println("No Tag");
//                 break;
//             case RedAmp:
//                 System.out.println("Red Amp");
//                 break;
//             case RedCenterStage:
//                 System.out.println("Red Center Stage");
//                 break;
//             case RedLeftHumanPlayer:
//                 System.out.println("Red Left Human Player");
//                 break;
//             case RedLeftStage:
//                 System.out.println("Red Left Stage");
//                 break;
//             case RedRightHumanPlayer:
//                 System.out.println("Red Right Human Player");
//                 break;
//             case RedRightStage:
//                 System.out.println("Red Right Stage");
//                 break;
//             case RedSpeaker:
//                 System.out.println("Red Speaker");
//                 break;
//             case RedSpeakerOffset:
//                 System.out.println("Red Speaker Offset");
//                 break;
//             default:
//                 System.out.println("No Tag Idenfied");
//                 break;

//         }

//     }

// }
