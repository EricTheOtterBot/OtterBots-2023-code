package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Lift;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class TeleopLift extends CommandBase {
    Lift s_ClawLift;
    DoubleSupplier extensionSup;
    DoubleSupplier liftSup;
    BooleanSupplier rotateSup;
    BooleanSupplier closeSup;
    BooleanSupplier openSup;

    
    public TeleopLift(Lift s_ClawLift, DoubleSupplier extensionSup, DoubleSupplier liftSup) {
        this.s_ClawLift = s_ClawLift;
        addRequirements(s_ClawLift);
        
        this.extensionSup = extensionSup;
        this.liftSup = liftSup;
    }
    
    @Override
    public void execute() {
        /* Get Values, Deadband*/
        double extensionVal = MathUtil.applyDeadband(extensionSup.getAsDouble(), Constants.stickDeadband);
        double liftVal = MathUtil.applyDeadband(liftSup.getAsDouble(), Constants.stickDeadband);

        /* Send control signals to subsystem */
        s_ClawLift.default_control(extensionVal, liftVal);
    }
}
