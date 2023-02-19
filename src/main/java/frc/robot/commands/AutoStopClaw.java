package frc.robot.commands;

import frc.robot.subsystems.Claw;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class AutoStopClaw extends CommandBase {
    Claw s_Claw;
    DoubleSupplier extensionSup;
    DoubleSupplier liftSup;
    BooleanSupplier rotateSup;
    BooleanSupplier closeSup;
    BooleanSupplier openSup;

    
    public AutoStopClaw(Claw s_Claw) {
        this.s_Claw = s_Claw;
        addRequirements(s_Claw);
    }
    
    @Override
    public void execute() {
        s_Claw.auto_stop();
    }


    @Override
    public boolean isFinished() {
        return true;
    }
        
}
