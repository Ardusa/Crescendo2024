package frc.robot.Commands.Lights;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Subsystems.Lighting;;

public class LightCMD extends InstantCommand {
    private final Lighting mLighting = Lighting.getInstance();

    private double PWMVal;

    public LightCMD(double color) {
        PWMVal = color;
        this.addRequirements(mLighting);
    }

    @Override
    public void initialize() {}
    
    @Override
    public void execute() {
        mLighting.setLights(PWMVal);
    }
}