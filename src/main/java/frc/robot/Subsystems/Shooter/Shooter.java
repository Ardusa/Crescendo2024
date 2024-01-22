// package frc.robot.Subsystems.Shooter;

// import com.ctre.phoenix6.signals.NeutralModeValue;

// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants;
// import frc.robot.Robot;
// import frc.robot.Commands.Shooter.Intake;
// import frc.robot.Custom.LoggyThings.LoggyTalonFX;

// public class Shooter extends SubsystemBase {
//     private static Shooter mInstance;
//     private LoggyTalonFX beltMotorRight, beltMotorLeft;

//     public enum Target {
//         kSpeaker,
//         kAmp,
//         None
//     }

//     private boolean holding;

//     public static Shooter getInstance() {
//         if (mInstance == null) {
//             mInstance = new Shooter();
//         }

//         return mInstance;
//     }

//     private Shooter() {
//         beltMotorRight = new LoggyTalonFX(Constants.BeltConstants.beltMotorRight, true);
//         beltMotorLeft = new LoggyTalonFX(Constants.BeltConstants.beltMotorLeft, true);
//         beltMotorRight.setNeutralMode(NeutralModeValue.Coast);
//         beltMotorLeft.setNeutralMode(NeutralModeValue.Coast);
//         // beltMotorLeft.setControl(new Follower(beltMotorRight.getDeviceID(), false));
//         holding = false;
//     }

//     public void intakeSpeed(double speed) {
//         beltMotorRight.set(-speed);
//     }

//     public void intake() {
//         beltMotorRight.set(Constants.BeltConstants.kBeltIntakeSpeed);
//     }

//     public void shoot(double speed, double speed2) {
//         beltMotorRight.set(speed);
//         beltMotorLeft.set(speed2);
//         setHolding(false);
//         SmartDashboard.putNumber("Left Power", speed);
//         SmartDashboard.putNumber("Right Power", speed2);
//     }

//     public void shoot(double speed) {
//         shoot(speed, speed);
//     }

//     public void stop() {
//         beltMotorRight.set(0);
//     }

//     public void setHolding(boolean holding) {
//         this.holding = holding;
//     }

//     public void toggleHolding() {
//         holding = !holding;
//     }

//     public boolean getHolding() {
//         return holding;
//     }

//     @Override
//     public void periodic() {
//         if (Robot.isReal()) {
//             SmartDashboard.putNumber("Velocity Left Belt", beltMotorRight.getVelocity().getValue());
//             SmartDashboard.putNumber("Velocity Right Belt", beltMotorLeft.getVelocity().getValue());
//             SmartDashboard.putNumber("Spin Delta Belt",
//                     beltMotorLeft.getVelocity().getValue() - beltMotorRight.getVelocity().getValue());
//         }
        
//         if (getCurrentCommand() instanceof Intake) {
//             if (beltMotorLeft.get() * Constants.BeltConstants.kBeltIntakeVelocityMax > beltMotorLeft.getVelocity()
//                     .getValue() + Constants.BeltConstants.beltBufferVelocity && Robot.isReal()) {
//                 setHolding(true);
//                 stop();
//             }
//         }
//     }
// }