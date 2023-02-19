package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.constraint.TrajectoryConstraint.MinMax;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class AutoBalance extends CommandBase {    
    private Swerve s_Swerve;  


    public AutoBalance(Swerve s_Swerve) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);
    }

    @Override
    public void execute() {
        /* Get Values, Deadband*/
        double translationVal = -0.8;
        double strafeVal = 0;
        double rotationVal = 0;

        s_Swerve.drive(
            new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed), 
            rotationVal * Constants.Swerve.maxAngularVelocity, 
            !false, 
            true
        );
    }

    @Override
    public boolean isFinished() {
        if(s_Swerve.getPitch() < -12){
            return true;
        }else {
            return false;
        } 
    }


    @Override
    public void end(boolean interrupted) {
        double translationVal = 0;
        double strafeVal = 0;
        double rotationVal = 0;

        s_Swerve.drive(
            new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed), 
            rotationVal * Constants.Swerve.maxAngularVelocity, 
            !false, 
            true
        );
    }
}
 