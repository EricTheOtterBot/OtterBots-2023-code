package frc.robot.commands;

import frc.robot.subsystems.Lift;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class AutoLowerLift extends CommandBase {
    Lift s_ClawLift;
    DoubleSupplier extensionSup;
    DoubleSupplier liftSup;
    BooleanSupplier rotateSup;
    BooleanSupplier closeSup;
    BooleanSupplier openSup;

    
    public AutoLowerLift(Lift s_ClawLift) {
        this.s_ClawLift = s_ClawLift;
        addRequirements(s_ClawLift);
    }
    
    @Override
    public void execute() {
        s_ClawLift.auto_lower();
    }
        
    @Override
    public boolean isFinished() {
        return s_ClawLift.get_bottom_limit();
    }

    @Override
    public void end(boolean interrupted) {
        s_ClawLift.auto_stop_raise();
    }


}
