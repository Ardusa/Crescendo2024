package frc.robot.Commands.Shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Subsystems.Shooter.Arm;

public class SetPoint extends Command {
    private Arm mArm;
    private double shooterExtensionSetpoint, armSetpoint;
    private double error = 0;
    private Timer timer;
    private boolean debugMode = true;

    /**
     * Set the shooter to a specific position
     * 
     * @param target in degrees of THE SHOOTER, not the extension bar
     */
    public SetPoint(double target) {
        timer = new Timer();
        mArm = Arm.getInstance();
        shooterExtensionSetpoint = target + Constants.ArmConstants.shooterOffset;
        armSetpoint = target;

        this.setName("Setpoint: " + armSetpoint + " degrees");
        this.addRequirements(mArm);
    }

    @Override
    public void initialize() {
        mArm.setArmTarget(shooterExtensionSetpoint);
        timer.restart();

        error = armSetpoint - mArm.getArmPosition();

        if (debugMode) {
            System.out.println("\n*************************** Debug Stats (initialize) ***************************");
            System.out.println("Shooter position: " + mArm.getArmPosition());
            System.out.println("Shooter target position: " + armSetpoint);
            System.out.println("Error: " + error);
            // System.out.println("Output: " + (error * (rateOfMotion / Constants.ArmConstants.kArmRangeOfMotion)));
            System.out.println("*************************** Debug Stats (initialize) ***************************\n");
        }
    }

    @Override
    public void execute() {
        if (!mArm.validSetpoint(armSetpoint)) {
            this.end(true);
        }

        mArm.setMotionMagic(armSetpoint);

        if (debugMode) {
            System.out.println("\n*************************** Debug Stats (execute) ***************************");
            System.out.println("Shooter position: " + mArm.getArmPosition());
            System.out.println("Shooter target position: " + armSetpoint);
            System.out.println("Error: " + error);
            // System.out.println("Output: " + (error * (rateOfMotion / Constants.ArmConstants.kArmRangeOfMotion)));
            System.out.println("*************************** Debug Stats (execute) ***************************\n");
        }
    }

    @Override
    public boolean isFinished() {
        return mArm.isInRangeOfTarget(armSetpoint);
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            SmartDashboard.putNumber("Velocity", 0);
        }
        mArm.stop();
        System.out.println("Shooter position (end of command): " + (mArm.getArmPosition()));
    }
}