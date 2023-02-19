package frc.robot.autos;

import frc.robot.Constants;
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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

public class testAuto extends SequentialCommandGroup {
    public testAuto(Swerve s_Swerve){
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
    

        // An example trajectory to follow.  All units in meters.
        Trajectory ericTrajectory1 =
            TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(0, 0, new Rotation2d(0)),
                // Pass through these two interior waypoints, making an 's' curve path
                List.of(
                new Translation2d(0.01, -0.1), 
                new Translation2d(0.0, -0.2), 
                new Translation2d(0.01, -0.3), 
                new Translation2d(0.0, -0.4), 
                new Translation2d(0.01, -0.5), 
                new Translation2d(0.0, -0.6), 
                new Translation2d(0.01, -0.7), 
                new Translation2d(0.0, -0.8), 
                new Translation2d(0.01, -0.9), 
                new Translation2d(0.0, -1)
                ),
                // End 3 meters straight ahead of where we started, facing forward
                new Pose2d(0.01, -1, new Rotation2d(0)),
                configB);

        Trajectory ericTrajectory2 =
            TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(0.01, -1, new Rotation2d(0)),
                // Pass through these two interior waypoints, making an 's' curve path
                List.of(
                    new Translation2d(0.1, -1.01), 
                    new Translation2d(0.2, -1.0), 
                    new Translation2d(0.3, -1.01), 
                    new Translation2d(0.4, -1.0), 
                    new Translation2d(0.5, -1.01), 
                    new Translation2d(0.6, -1.0), 
                    new Translation2d(0.7, -1.01), 
                    new Translation2d(0.8, -1.0), 
                    new Translation2d(0.9, -1.01), 
                    new Translation2d(1, -1.0)
                    ),
                // End 3 meters straight ahead of where we started, facing forward
                new Pose2d(1.01, -1, new Rotation2d(0)),
                configB);

        Trajectory ericTrajectory3 =
            TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(1.01, -1, new Rotation2d(0)),
                // Pass through these two interior waypoints, making an 's' curve path
                List.of(
                new Translation2d(1.0, -0.9), 
                new Translation2d(1.01, -0.8),
                new Translation2d(1.0, -0.7),
                new Translation2d(1.01, -0.6),
                new Translation2d(1.0, -0.5),
                new Translation2d(1.01, -0.4),
                new Translation2d(1.0, -0.3),
                new Translation2d(1.01, -0.2),
                new Translation2d(1.0, -0.1),
                new Translation2d(1.01, -0)
                ),
                // End 3 meters straight ahead of where we started, facing forward
                new Pose2d(1.0, -0.0, new Rotation2d(0)),
                configB);
    
        Trajectory ericTrajectory4 =
            TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(1, 0, new Rotation2d(0)),
                // Pass through these two interior waypoints, making an 's' curve path
                List.of(
                new Translation2d(0.9, 0.01), 
                new Translation2d(0.8, 0.0), 
                new Translation2d(0.7, 0.01), 
                new Translation2d(0.6, 0.0), 
                new Translation2d(0.5, 0.01), 
                new Translation2d(0.4, 0.0), 
                new Translation2d(0.3, 0.01), 
                new Translation2d(0.2, 0.0), 
                new Translation2d(0.1, 0.01), 
                new Translation2d(0.0, 0.0)
                ),
                // End 3 meters straight ahead of where we started, facing forward
                new Pose2d(0.01, 0.01, new Rotation2d(0)),
                configB);
    

        var thetaController =
            new ProfiledPIDController(
                Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand step1 =
            new SwerveControllerCommand(
                ericTrajectory1,
                s_Swerve::getPose,
                Constants.Swerve.swerveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                thetaController,
                s_Swerve::setModuleStates,
                s_Swerve);

        SwerveControllerCommand step2 =
            new SwerveControllerCommand(
                ericTrajectory2,
                s_Swerve::getPose,
                Constants.Swerve.swerveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                thetaController,
                s_Swerve::setModuleStates,
                s_Swerve);

        SwerveControllerCommand step3 =
            new SwerveControllerCommand(
                ericTrajectory3,
                s_Swerve::getPose,
                Constants.Swerve.swerveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                thetaController,
                s_Swerve::setModuleStates,
                s_Swerve);

        SwerveControllerCommand step4 =
            new SwerveControllerCommand(
                ericTrajectory4,
                s_Swerve::getPose,
                Constants.Swerve.swerveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                thetaController,
                s_Swerve::setModuleStates,
                s_Swerve);

        addCommands(
            new InstantCommand(() -> s_Swerve.resetOdometry(ericTrajectory4.getInitialPose())),
            step1,
            step2,
            step3,
            step4 
        );
    }
}