package frc.robot.autos;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Lift;
import frc.robot.commands.AutoBalance;
import frc.robot.commands.AutoRaiseLift;
import frc.robot.commands.stopSwerve;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;


public class testAutoClawLift extends SequentialCommandGroup {
    public testAutoClawLift(Swerve s_Swerve, Lift s_Lift){
        TrajectoryConfig configA =
            new TrajectoryConfig(
                    Constants.AutoConstants.kMaxSpeedMetersPerSecondA,
                    Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquaredA)
                .setKinematics(Constants.Swerve.swerveKinematics);


        TrajectoryConfig configB =
            new TrajectoryConfig(
                    Constants.AutoConstants.kMaxSpeedMetersPerSecondB,
                    Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquaredB)
                .setKinematics(Constants.Swerve.swerveKinematics);


        Trajectory ericTrajectory =
            TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(0, 0, new Rotation2d(0)),
                // Pass through these two interior waypoints, making an 's' curve path
                List.of(new Translation2d(0.2, 0), new Translation2d(0.4, -0.2)),
                // End 3 meters straight ahead of where we started, facing forward
                new Pose2d(0.6, 0, new Rotation2d(90)),
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

        Command autobalance = new AutoBalance(s_Swerve);
        Command raiseLift = new AutoRaiseLift(s_Lift);
        Command stopSwerve = new stopSwerve(s_Swerve);

        addCommands(
            new InstantCommand(() -> s_Swerve.resetOdometry(ericTrajectory.getInitialPose())),
            swerveControllerCommand,
            autobalance,
            stopSwerve,
            raiseLift
        );
    }
}