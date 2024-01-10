// package frc.robot.Subsystems.Shooter;

// import com.ctre.phoenix6.signals.NeutralModeValue;

// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants;
// import frc.robot.Robot;
// import frc.robot.Custom.LoggyThings.LoggyTalonFX;

// public class Shooter extends SubsystemBase {
//     private static Shooter mInstance;
//     private LoggyTalonFX beltMotor;

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
//         beltMotor = new LoggyTalonFX(Constants.BeltConstants.beltMotorID, false);
//         beltMotor.setNeutralMode(NeutralModeValue.Brake);
//         holding = false;
//     }

//     public void intakeSpeed(double speed) {
//         if (!Constants.BeltConstants.intakeIsPositive) {
//             speed = -speed;
//         }

//         beltMotor.set(speed);
//     }

//     public void intake() {
//         beltMotor.set(Constants.BeltConstants.kBeltIntakeSpeed);
//     }

//     public void shoot(double speed) {
//         if (Constants.BeltConstants.intakeIsPositive) {
//             speed = -speed;
//         }

//         beltMotor.set(speed);
//     }

//     public void stop() {
//         beltMotor.set(0);
//     }

//     public void setHolding(boolean hold) {
//         holding = hold;
//     }

//     public void toggleHolding() {
//         holding = !holding;
//     }

//     public boolean getHolding() {
//         return holding;
//     }

//     @Override
//     public void periodic() {
//         if (!(Math.abs(beltMotor.get()) < 0.01) && Robot.isReal()) {

//             System.out.println("Belt velocity" + beltMotor.getVelocity().getValue());
//             System.out.println("Belt speed" + beltMotor.get());

//             if (beltMotor.get() == Constants.BeltConstants.kBeltIntakeSpeed
//                     && beltMotor.getVelocity().getValue() < (Constants.BeltConstants.kBeltIntakeVelocity * 0.8)) {
//                 setHolding(true);
//                 stop();
//             }
//         }
//         if (beltMotor.get() == Constants.BeltConstants.kBeltSpeedSpeaker || beltMotor.get() == Constants.BeltConstants.kBeltSpeedAmp) {
//             setHolding(false);
//         }
//     }
// }