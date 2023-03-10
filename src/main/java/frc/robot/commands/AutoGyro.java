package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class AutoGyro extends CommandBase {    
    private Swerve s_Swerve;
    private double translation; 
    private double strafe; 
    private double rotation; 
    private double gyroTarget;
    private boolean greaterThan;
    private boolean IsRoll; 
    private boolean IsYaw;

    public AutoGyro(Swerve s_Swerve, double translation, double strafe, double rotation, 
                        double gyroTarget, boolean greaterThan, boolean IsRoll, boolean IsYaw) {
        this.s_Swerve = s_Swerve;
        this.translation = translation; 
        this.strafe = strafe;
        this.rotation = rotation;
        this.gyroTarget = gyroTarget;
        this.greaterThan = greaterThan;
        this.IsRoll = IsRoll; 
        this.IsYaw = IsYaw;
        addRequirements(s_Swerve);
    }

    @Override
    public void execute() {
        /* Get Values, Deadband*/
        double translationVal = translation;
        double strafeVal = strafe;
        double rotationVal = rotation;

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
        if(IsRoll) {
            gyroAngle = s_Swerve.getRoll();
        } 
        else if(IsYaw) {
            gyroAngle = s_Swerve.getYaw2();
        }
        else {
            gyroAngle = s_Swerve.getPitch();
        }
        if(gyroAngle < gyroTarget && !greaterThan) {
            return true;
        } 
        else if(gyroAngle > gyroTarget && greaterThan) {
            return true;
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
 