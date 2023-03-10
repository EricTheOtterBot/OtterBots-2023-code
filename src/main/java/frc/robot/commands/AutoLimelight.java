package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.LimelightVision;
import frc.robot.subsystems.Swerve;

public class AutoLimelight extends CommandBase {
    
    LimelightVision s_Limelight;
    double averageRotation;
    Swerve s_Swerve;
    int pipelineNumber;
    Timer m_timer;
    double target_time;
    boolean timer_started;
    
    public AutoLimelight(LimelightVision s_Limelight, Swerve s_Swerve, int pipelineNumber, Timer m_timer) {
        this.s_Limelight = s_Limelight;
        this.averageRotation = 0.0;
        this.s_Swerve = s_Swerve;
        this.pipelineNumber = pipelineNumber;
        this.m_timer = m_timer;
    }



    @Override
    public void initialize() {
        s_Limelight.setPipeline(pipelineNumber);
    }

    @Override
    public void execute() {
        averageRotation = s_Limelight.calculateAverage();
        if(!timer_started) {
            s_Swerve.drive(
                new Translation2d(0, 0), 
                -averageRotation / 100 * Constants.Swerve.maxAngularVelocity, 
                !false, 
                true
            );
        } else {
            s_Swerve.drive(
                new Translation2d(0, 0), 
                0.0, 
                !false, 
                true
            );
        }
    }

    @Override
    public boolean isFinished() {
        boolean targetExists = s_Limelight.targetExists();

        // start a timer if centered
        if(averageRotation < 2 && averageRotation > -2 && targetExists) {
            if(timer_started == false) {
                target_time = m_timer.get() + 0.3;
            }
            timer_started = true;
        } else {
            timer_started = false;
        }

        // check if we've been centered for 0.25s 
        if(m_timer.get() >= target_time && timer_started == true) {
            return true;
        } else {
            return false;
        }
    }

    @Override
    public void end(boolean Interrupted) {
        s_Swerve.drive(
            new Translation2d(0, 0), 
            0, 
            !false, 
            true
        );
    }
}
