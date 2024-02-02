package frc.robot.Subsystems;

import java.util.function.Consumer;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Custom.LoggyThings.LoggyTalonFX;

public class Intake extends SubsystemBase {
    private static Intake mInstance;

    private LoggyTalonFX intakeMotor;
    private DoubleSolenoid intakeSolenoid;
    private boolean holding;
    private boolean deployed;
    public static Consumer<Boolean> deploy = (bool) -> Intake.getInstance().toggleDeploy();
    public static Runnable toggleDeploy = () -> Intake.getInstance().toggleHolding();

    public static Intake getInstance() {
        if (mInstance == null) {
            mInstance = new Intake();
        }
        return mInstance;
    }

    @Override
    public void periodic() {
        if (deployed) {
            intakeSolenoid.set(DoubleSolenoid.Value.kForward);
            intakeMotor.set(1);
        } else {
            intakeSolenoid.set(DoubleSolenoid.Value.kReverse);
            intakeMotor.set(0);
        }
    }

    private Intake() {
        intakeMotor = new LoggyTalonFX(Constants.IntakeConstants.intakeMotorID, false);

        intakeSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, Constants.IntakeConstants.intakeSolenoidFwdID,
                Constants.IntakeConstants.intakeSolenoidRevID);
    }

    public void setHolding(boolean holding) {
        this.holding = holding;
    }

    public void toggleHolding() {
        holding = !holding;
    }

    public boolean getHolding() {
        return holding;
    }

    public void setDeployed(boolean deployed) {
        this.deployed = deployed;
    }

    public void toggleDeploy() {
        deployed = !deployed;
    }

}
