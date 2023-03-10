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

public class AutoOnePiece extends SequentialCommandGroup {
    public AutoOnePiece(Swerve s_Swerve, Lift s_ClawLift, Claw s_Claw, Timer m_timer){

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



        Command waitcommand = new WaitCommand(0.2);
        Command waitcommand100 = new WaitCommand(0.1);

        Command extendcommand = new EricAutoExtend(s_ClawLift, -1, true, true, 0, m_timer);
        Command openclawcommand = new EricAutoClaw(s_Claw, 1, true, 0.3, m_timer);
        Command stopclawcommand = new EricAutoClaw(s_Claw, 0, true, 0, m_timer);
        Command retractandlowercommand = new EricAutoRetractAndLower(s_ClawLift, 1, -1, true, false, false, 0, m_timer);
        Command as0_3command = new AutoSpeed(s_Swerve, 0.3, 0, 0,4, m_timer, false);
        Command ag0_2command = new AutoGyro(s_Swerve, 0, 0, 0.3, 165, true, false, true);

        Command liftcommand = new EricAutoLift(s_ClawLift, 1, true, true, 0, m_timer);


        addCommands(
            new InstantCommand(() -> s_Swerve.resetOdometry(ericTrajectory.getInitialPose())).alongWith(waitcommand100),
            liftcommand, 
            extendcommand, 
            openclawcommand, 
            waitcommand, 
            stopclawcommand, 
            retractandlowercommand,
            as0_3command, 
            ag0_2command



        );
    }
}