package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class AutoBalance2 extends CommandBase {    
    private Swerve s_Swerve;    

    public AutoBalance2(Swerve s_Swerve) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);
    }

    @Override
    public void execute() {
        /* Get Values, Deadband*/
        double translationVal = -0.15;
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
        if(s_Swerve.getPitch() >= -10){
            return true;
        }else {
            return false;
        } 
    }


    @Override
    public void end(boolean interrupted) {
        double translationVal = -0.2;
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
 