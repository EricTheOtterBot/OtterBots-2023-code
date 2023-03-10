package frc.robot.autos;

import frc.robot.Constants;
import frc.robot.commands.*;
import frc.robot.oi.LimeLight;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.LimelightVision;
import frc.robot.subsystems.Swerve;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class AutoHighCubeL extends SequentialCommandGroup {
    public AutoHighCubeL(LimelightVision s_Limelight, Swerve s_Swerve, Claw s_Claw, Lift s_Lift, Timer m_timer){

    

        TrajectoryConfig configB =
            new TrajectoryConfig(
                    Constants.AutoConstants.kMaxSpeedMetersPerSecondB,
                    Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquaredB)
                .setKinematics(Constants.Swerve.swerveKinematics);

        Trajectory ericTrajectory =
            TrajectoryGenerator.generateTrajectory(
                new Pose2d(0, 0, new Rotation2d(0)),
                List.of(new Translation2d(0.2, 0.0), new Translation2d(0.8, -1)),
                new Pose2d(3, -1, new Rotation2d(0)),
                configB);

        var thetaController =
            new ProfiledPIDController(
                Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand swerveControllerCommand =
            new SwerveControllerCommand(
                ericTrajectory,
                s_Swerve::getPose,
                Constants.Swerve.swerveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                thetaController,
                s_Swerve::setModuleStates,
                s_Swerve);


        Command waitcommand = new WaitCommand(0.5);
        Command waitcommand2 = new WaitCommand(0.5);
        Command waitcommand100 = new WaitCommand(0.1);

        Command raisecommand2 = new EricAutoRetractAndLower(s_Lift, 1, -1, false, false, false, 1, m_timer);
        Command raisecommand4 = new EricAutoLift(s_Lift, 1, true, true, 0, m_timer);
        Command raisecommand3 = new EricAutoLift(s_Lift, 1, true, false, 0.6, m_timer);

        Command stopswervecommand = new AutoSpeed(s_Swerve, 0, 0, 0, 0, m_timer, false);

        Command lowercommand = new EricAutoLift(s_Lift, -1, false, false, 0.3, m_timer);
        Command stopswervecommand2 = new AutoSpeed(s_Swerve, 0, 0, 0, 0, m_timer, false);

        Command extendcommand = new EricAutoExtend(s_Lift, -1, true, true, 0, m_timer);
        Command extendcommand2 = new EricAutoExtend(s_Lift, -0.4, true, false, 2.4, m_timer);

        Command retractcommand = new EricAutoExtend(s_Lift, 1, false, true, 0, m_timer);

        Command openclawcommand = new EricAutoClaw(s_Claw, 1, true, 0.3, m_timer);
        Command openclawcommand2 = new EricAutoClaw(s_Claw, 1, true, 0, m_timer);

        Command closeclawcommand = new EricAutoClaw(s_Claw, -1, true, 0.5, m_timer);

        Command retractandlowercommand = new EricAutoRetractAndLower(s_Lift, 1, -1, false, false, false, 0.4, m_timer);
        Command retractandlowercommand2 = new EricAutoRetractAndLower(s_Lift, 1, -1, true, false, false, 0, m_timer);

        Command stopliftcommand2 = new EricAutoLift(s_Lift, 0, true, false, 0, m_timer);
        Command stopliftcommand3 = new EricAutoLift(s_Lift, 0, true, false, 0, m_timer);

        Command extendcommand3 = new EricAutoExtend(s_Lift, -1, true, true, 0, m_timer);
        Command openclawcommand4 = new EricAutoClaw(s_Claw, 1, true, 0.3, m_timer);
        Command stopclawcommand = new EricAutoClaw(s_Claw, 0, true, 0, m_timer);

        Command retractandlowercommand3 = new EricAutoRetractAndLower(s_Lift, 1, -1, true, false, false, 0, m_timer);
        Command retractandlowercommand4 = new EricAutoRetractAndLower(s_Lift, 1, -1, false, false, false, 1, m_timer);


        Command as0_3command = new AutoSpeed(s_Swerve, 0.4, 0.0, 0.0, 1.8, m_timer, false);
        Command as0_1command2 = new AutoSpeed(s_Swerve, 0.1, 0, 0, 0.3, m_timer, false);
        Command as0_2command2 = new AutoSpeed(s_Swerve, 0.3, 0, 0, 1.4, m_timer, true);
        Command as_0_2command = new AutoSpeed(s_Swerve, -0.3, 0, 0, 1.0, m_timer, true);
        Command ag0_3gp155command = new AutoGyro(s_Swerve, 0.3, 0.0, 0.4, 155, true, false, true);
        Command ag0_3lp20command = new AutoGyro(s_Swerve, 0, 0, -0.4, 20, false, false, true);
        Command alcommand = new AutoLimelight(s_Limelight, s_Swerve, 1, m_timer);
        Command as0_2translatecommand = new AutoSpeed(s_Swerve, 0.2, 0.2, 0, 0.5, m_timer, false);
        Command stopswervecommand3 = new AutoSpeed(s_Swerve, 0, 0, 0, 0, m_timer, false);
        Command as0_1command = new AutoSpeed(s_Swerve, -0.1, 0, 0, 0.2, m_timer, false);

        Command liftcommand = new EricAutoLift(s_Lift, 1, true, true, 0, m_timer);



        addCommands(
            //new InstantCommand(() -> s_Swerve.resetOdometry(ericTrajectory.getInitialPose())),
            new InstantCommand(() -> s_Swerve.zeroGyro()).alongWith(waitcommand100),
            liftcommand,
            extendcommand3, 
            openclawcommand4, 
            stopclawcommand, 
            retractandlowercommand4, 
            as0_1command2,
            as0_2translatecommand,
            retractandlowercommand2.alongWith(as0_3command),
            ag0_3gp155command,
            alcommand.alongWith(openclawcommand2), 
            as0_2command2,
            stopswervecommand, 
            closeclawcommand, 
            raisecommand3,
            retractcommand,
            as_0_2command,
            ag0_3lp20command,
            as0_1command, 
            // as0_2translatecommand2,
            // as0_5command.alongWith(raisecommand2),
            stopswervecommand2
        );
    }
}