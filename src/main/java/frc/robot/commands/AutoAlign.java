package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class AutoAlign extends CommandBase {    
    private Swerve s_Swerve;
    private double translation; 
    private double strafe; 
    private double rotation;

    public AutoAlign(Swerve s_Swerve, double translation, double strafe, double rotation) {
        this.s_Swerve = s_Swerve;
        this.translation = translation; 
        this.strafe = strafe;
        this.rotation = rotation;
        addRequirements(s_Swerve);
    }

    @Override
    public void execute() {
        /* Get Values, Deadband*/
        double translationVal = translation;
        double strafeVal = strafe;
        double rotationVal = rotation;
        double gyroAngle;

        gyroAngle = s_Swerve.getYaw2();

        if(gyroAngle > 0) {
            rotationVal = -rotationVal;
        }

        s_Swerve.drive(
            new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed), 
            rotationVal * Constants.Swerve.maxAngularVelocity, 
            !false, 
            true
        );
    }

    @Override
    public boolean isFinished() {
        double gyroAngle;
        gyroAngle = s_Swerve.getYaw2();

        if(gyroAngle > -5.0) {
            if(gyroAngle < 5.0) {
                return true;
            }
            else {
                return false;
            }
        }
        else {
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
 