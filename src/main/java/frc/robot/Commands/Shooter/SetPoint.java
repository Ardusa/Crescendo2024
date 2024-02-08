package frc.robot.Commands.Shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Shooter.Arm;

public class SetPoint extends Command {
    private Arm mArm;
    private double armSetpoint;
    private double error = 0;
    private Timer timer;
    private boolean debugMode = false;

    /**
     * Set the shooter to a specific position
     * 
     * @param target in degrees of THE SHOOTER, not the extension bar
     */
    public SetPoint(double target) {
        timer = new Timer();
        mArm = Arm.getInstance();
        armSetpoint = target;

        this.setName("Setpoint: " + armSetpoint + " degrees");
        this.addRequirements(mArm);
    }

    @Override
    public void initialize() {
        mArm.setArmTarget(armSetpoint);
        timer.restart();

        error = armSetpoint - mArm.getArmPosition();
        SmartDashboard.putNumber("Arm/Arm Position Error", error);
        SmartDashboard.putNumber("Arm/Setpoint", armSetpoint);

        if (debugMode) {
            System.out.println("\n*************************** Debug Stats (initialize) ***************************");
            System.out.println("Shooter position: " + mArm.getArmPosition());
            System.out.println("Shooter target position: " + armSetpoint);
            System.out.println("Error: " + error);
            System.out.println("*************************** Debug Stats (initialize) ***************************\n");
        }
        mArm.setArmTarget(armSetpoint);
        mArm.setMotionMagic(armSetpoint);
    }
    
    @Override
    public void execute() {
        if (!mArm.validSetpoint(armSetpoint)) {
            this.end(true);
        }

        error = armSetpoint - mArm.getArmPosition();

        SmartDashboard.putNumber("Arm/Arm Position Error", error);
        SmartDashboard.putNumber("Arm/Velocity", mArm.getVelocity());

        if (debugMode) {
            System.out.println("\n*************************** Debug Stats (execute) ***************************");
            System.out.println("Shooter position: " + mArm.getArmPosition());
            System.out.println("Shooter target position: " + armSetpoint);
            System.out.println("Error: " + error);
            System.out.println("*************************** Debug Stats (execute) ***************************\n");
        }
    }

    @Override
    public boolean isFinished() {
        return false;

        // if (Robot.isSimulation()) {
        //     return timer.get() > 0.3;
        // } else {
        //     return mArm.isInRangeOfTarget(armSetpoint);
        // }
        // return timer.get() > 0.3;
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            SmartDashboard.putNumber("Arm/Velocity", 0);
        }
    }
}