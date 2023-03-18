package frc.robot.commands;

import frc.robot.subsystems.Claw;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class TeleopClaw extends CommandBase {
    Claw s_Claw;
    DoubleSupplier extensionSup;
    DoubleSupplier liftSup;
    BooleanSupplier rotateFSup;
    BooleanSupplier rotateRSup;
    BooleanSupplier closeSup;
    BooleanSupplier openSup;

    
    public TeleopClaw(Claw s_Claw,
                      BooleanSupplier rotateFSup, BooleanSupplier rotateRSup, 
                      BooleanSupplier closeSup, BooleanSupplier openSup) {
        this.s_Claw = s_Claw;
        addRequirements(s_Claw);

        this.rotateFSup = rotateFSup;
        this.rotateRSup = rotateRSup;
        this.closeSup = closeSup;
        this.openSup = openSup;
    }
    
    @Override
    public void execute() {
        /* Get Values */
        boolean rotateF = rotateFSup.getAsBoolean();
        boolean rotateR = rotateRSup.getAsBoolean(); 
        boolean close = closeSup.getAsBoolean();
        boolean open = openSup.getAsBoolean();

        /* Send control signals to subsystem */
        s_Claw.default_control(rotateF, rotateR, close, open);
    }
}
