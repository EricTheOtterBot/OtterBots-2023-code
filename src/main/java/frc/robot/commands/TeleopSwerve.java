package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class TeleopSwerve extends CommandBase {    
    private Swerve s_Swerve;    
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private BooleanSupplier robotCentricSup;
    private BooleanSupplier targetLockSup;
    private DoubleSupplier targetRotationSup;
    private DoubleSupplier speedControlSup;

    public TeleopSwerve(Swerve s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup,
     DoubleSupplier rotationSup, BooleanSupplier robotCentricSup, BooleanSupplier targetLockSup,
     DoubleSupplier targetRotationSup, DoubleSupplier speedControlSup) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.robotCentricSup = robotCentricSup;
        this.targetLockSup = targetLockSup;
        this.targetRotationSup = targetRotationSup;
        this.speedControlSup = speedControlSup;
    }

    @Override
    public void execute() {
        /* Get Values, Deadband*/
        double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband);
        double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband);
        double rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.stickDeadband);
        double targetRotation = MathUtil.applyDeadband(targetRotationSup.getAsDouble(), 0.5);
        double speedControlVal = MathUtil.applyDeadband(speedControlSup.getAsDouble(), Constants.stickDeadband);

        if(speedControlVal < 0.4) {
            speedControlVal = 0.4;
        }   

        /* Drive */
        if(targetLockSup.getAsBoolean() == true) {
            rotationVal = targetRotation;
        }
        s_Swerve.drive(
            new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed).times(speedControlVal), 
            rotationVal * Constants.Swerve.maxAngularVelocity, 
            !robotCentricSup.getAsBoolean(), 
            true
        );
    }
}
