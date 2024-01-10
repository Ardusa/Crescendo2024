// package frc.robot.Custom;

// import com.ctre.phoenix6.configs.CANcoderConfiguration;
// import com.ctre.phoenix6.configs.TalonFXConfiguration;
// import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
// import com.ctre.phoenix6.signals.SensorDirectionValue;

// import frc.robot.Constants;

// public final class CTREConfigs {
//     public TalonFXConfiguration swerveAngleFXConfig, swerveDriveFXConfig;
//     public CANcoderConfiguration swerveCanCoderConfig;

//     public CTREConfigs(){
//         swerveAngleFXConfig = new TalonFXConfiguration();
//         swerveDriveFXConfig = new TalonFXConfiguration();
//         swerveCanCoderConfig = new CANcoderConfiguration();

//         /* Swerve Angle Motor Configurations */
//         swerveAngleFXConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
//         swerveAngleFXConfig.CurrentLimits.SupplyCurrentLimit = Constants.Swerve.angleContinuousCurrentLimit;
//         swerveAngleFXConfig.CurrentLimits.SupplyTimeThreshold = Constants.Swerve.anglePeakCurrentDuration;
//         swerveAngleFXConfig.CurrentLimits.SupplyCurrentThreshold = Constants.Swerve.anglePeakCurrentLimit;

//         swerveAngleFXConfig.Slot0.kP = Constants.Swerve.angleKP;
//         swerveAngleFXConfig.Slot0.kI = Constants.Swerve.angleKI;
//         swerveAngleFXConfig.Slot0.kD = Constants.Swerve.angleKD;
//         // swerveAngleFXConfig.Slot0.kF = Constants.Swerve.angleKF;




//         /* Swerve Drive Motor Configuration */
//         swerveDriveFXConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
//         swerveDriveFXConfig.CurrentLimits.SupplyCurrentLimit = Constants.Swerve.driveContinuousCurrentLimit;
//         swerveDriveFXConfig.CurrentLimits.SupplyTimeThreshold = Constants.Swerve.drivePeakCurrentDuration;
//         swerveDriveFXConfig.CurrentLimits.SupplyCurrentThreshold = Constants.Swerve.drivePeakCurrentLimit;

//         swerveDriveFXConfig.Slot0.kP = Constants.Swerve.driveKP;
//         swerveDriveFXConfig.Slot0.kI = Constants.Swerve.driveKI;
//         swerveDriveFXConfig.Slot0.kD = Constants.Swerve.driveKD;
//         // swerveDriveFXConfig.Slot0.kF = Constants.Swerve.driveKF;
//         swerveDriveFXConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = Constants.Swerve.openLoopRamp;
//         swerveDriveFXConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = Constants.Swerve.closedLoopRamp;
        
//         /* Swerve CANCoder Configuration */
//         swerveCanCoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
//         swerveCanCoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.valueOf(Constants.Swerve.canCoderInvert ? 1 : 0);
//         // swerveCanCoderConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
//         // swerveCanCoderConfig.sensorTimeBase = SensorTimeBase.PerSecond;
//     }
// }