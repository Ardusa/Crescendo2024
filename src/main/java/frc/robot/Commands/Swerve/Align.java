package frc.robot.Commands.Swerve;

import java.util.LinkedList;
import java.util.Queue;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Subsystems.Swerve.Swerve;
import frc.robot.Subsystems.Swerve.SwerveRequest;
import frc.robot.Subsystems.Swerve.SwerveModule.DriveRequestType;

public class Align extends Command {
    private Swerve mSwerve;
    private SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(Constants.SwerveConstants.kSpeedAt12VoltsMetersPerSecond * 0.08)
            .withRotationalDeadband(0)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private PIDController mPID = new PIDController(0.08, 0.05, 0.01);
    private DoubleSupplier leftX, leftY;
    private double xSpeed, ySpeed, MaxSpeed;

    private final NetworkTableEntry tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx");
    private final int positionsRecorded = 10;
    private Queue<Double> noteHist;
    private double runningSum;

    public Align(DoubleSupplier leftX, DoubleSupplier leftY, double MaxSpeed) {
        this.mSwerve = Swerve.getInstance();
        this.addRequirements(mSwerve);

        drive = new SwerveRequest.FieldCentric()
                .withDeadband(Constants.SwerveConstants.kSpeedAt12VoltsMetersPerSecond * 0.08)
                .withRotationalDeadband(0)
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

        this.leftX = leftX;
        this.leftY = leftY;
        this.MaxSpeed = MaxSpeed;
    }

    private double getAverageTX() {
        double x = tx.getDouble(0);
        noteHist.add(x);
        runningSum += x;
        if (noteHist.size() > positionsRecorded) {
            runningSum -= noteHist.remove();
        }
        return runningSum / noteHist.size();
    }

    @Override
    public void initialize() {
        noteHist = new LinkedList<>();
        mPID.setIntegratorRange(-1.5, 1.5);
        mPID.setIZone(20);
    }

    @Override
    public void execute() {
        xSpeed = leftX.getAsDouble();
        ySpeed = leftY.getAsDouble();
        mSwerve.setControl(drive.withVelocityY(-xSpeed * MaxSpeed) // Drive forward with negative Y (forward)
                .withVelocityX(-ySpeed * MaxSpeed) // Drive left with negative X (left)
                .withRotationalRate(mPID.calculate(getAverageTX()))// Drive counterclockwise with negative X (left)
        );
    }

    @Override
    public boolean isFinished() {
        return Math.abs(tx.getDouble(0)) < 0.2;
    }
}