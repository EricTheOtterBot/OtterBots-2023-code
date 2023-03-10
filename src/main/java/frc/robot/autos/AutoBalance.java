package frc.robot.autos;

import frc.robot.Constants;
import frc.robot.commands.*;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Lift;
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
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class AutoBalance extends SequentialCommandGroup {
    public AutoBalance(Swerve s_Swerve, Claw s_Claw, Lift s_Lift, Timer m_timer){

        TrajectoryConfig configB =
            new TrajectoryConfig(
                    Constants.AutoConstants.kMaxSpeedMetersPerSecondB,
                    Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquaredB)
                .setKinematics(Constants.Swerve.swerveKinematics);

        Trajectory ericTrajectory =
            TrajectoryGenerator.generateTrajectory(
                new Pose2d(0, 0, new Rotation2d(0)),
                List.of(new Translation2d(-1, 0.1)),
                new Pose2d(-0.5, 0, new Rotation2d(180)),
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




        Command waitcommand2 = new WaitCommand(0.1);
       

        Command extendcommand = new EricAutoExtend(s_Lift, -1, true, true, 0, m_timer);
        Command openclawcommand = new EricAutoClaw(s_Claw, 1, true, 0.3, m_timer);
        Command stopclawcommand = new EricAutoClaw(s_Claw, 0, true, 0, m_timer);
        Command retractandlowercommand = new EricAutoRetractAndLower(s_Lift, 1, -1, true, false, false, 0, m_timer);
        Command as0_1command = new AutoSpeed(s_Swerve, 0.1, 0.0, 0.0, 0.1, m_timer, false);
        Command as0_2command = new AutoSpeed(s_Swerve, 0.2, 0.0, 0.0, 0.1, m_timer, false);
        Command as0_3command = new AutoSpeed(s_Swerve, 0.3, 0.0, 0.0, 0.1, m_timer, false);
        Command as0_4command = new AutoSpeed(s_Swerve, 0.4, 0.0, 0.0, 0.1, m_timer, false);
        Command as0_5command = new AutoSpeed(s_Swerve, 0.5, 0.0, 0.0, 0.1, m_timer, false);
        Command as0_6command = new AutoSpeed(s_Swerve, 0.6, 0.0, 0.0, 0.1, m_timer, false);
        Command as0_7command = new AutoSpeed(s_Swerve, 0.7, 0.0, 0.0, 0.1, m_timer, false);
        Command ag0_75ln15command = new AutoGyro(s_Swerve, 0.75, 0.0, 0.0, -15, false, true, false);
        Command ag0_3gn5command = new AutoGyro(s_Swerve, 0.3, 0.0, 0.0, -5, true, true, false);
        Command as0_3command2 = new AutoSpeed(s_Swerve, 0.3, 0.0, 0.0, 1.2, m_timer, false);
        Command ag0command = new AutoAlign(s_Swerve, 0.0, 0.0, 0.1);
        Command as_0_1command = new AutoSpeed(s_Swerve, -0.1, 0.0, 0.0, 0.1, m_timer, false);
        Command as_0_2command = new AutoSpeed(s_Swerve, -0.2, 0.0, 0.0, 0.1, m_timer, false);
        Command as_0_3command = new AutoSpeed(s_Swerve, -0.3, 0.0, 0.0, 0.1, m_timer, false);
        Command as_0_4command = new AutoSpeed(s_Swerve, -0.4, 0.0, 0.0, 0.1, m_timer, false);
        Command as_0_5command = new AutoSpeed(s_Swerve, -0.5, 0.0, 0.0, 0.1, m_timer, false);
        Command as_0_6command = new AutoSpeed(s_Swerve, -0.6, 0.0, 0.0, 0.1, m_timer, false);
        Command as_0_7command = new AutoSpeed(s_Swerve, -0.7, 0.0, 0.0, 0.1, m_timer, false);
        Command ag_0_75gp15command = new AutoGyro(s_Swerve, -0.75, 0.0, 0.0, 15, true, true, false);
        Command as_0_6command2 = new AutoSpeed(s_Swerve, -0.6, 0.0, 0.0, 0.5, m_timer, false);
        Command ag_0_1lp10command = new AutoGyro(s_Swerve, -0.1, 0.0, 0.0, 7, false, true, false);
        Command as0command = new AutoSpeed(s_Swerve, 0.0, 0.0, 0.0, 0.2, m_timer, false);
        Command as0command2 = new AutoSpeed(s_Swerve, 0.1, 0.0, 0.0, 0.3, m_timer, false);

        Command liftandupcommand = new EricAutoRetractAndLower(s_Lift, -1, 1, true, true, true, 0, m_timer);



        addCommands(
            // new InstantCommand(() -> s_Swerve.resetOdometry(ericTrajectory.getInitialPose())),
            new InstantCommand(() -> s_Swerve.zeroGyro()).alongWith(waitcommand2),
            liftandupcommand, 
            openclawcommand,
            stopclawcommand,
            retractandlowercommand, 
            as0_1command, 
            as0_2command, 
            as0_3command, 
            as0_4command,
            as0_5command, 
            as0_6command,
            as0_7command,
            ag0_75ln15command, 
            ag0_3gn5command, 
            as0_3command2,
            ag0command,
            as_0_1command,
            as_0_2command,
            as_0_3command,
            as_0_4command,
            as_0_5command,
            as_0_6command,
            as_0_7command,
            ag_0_75gp15command,
            as_0_6command2, 
            ag_0_1lp10command, 
            as0command,
            as0command2

        );
    }
}