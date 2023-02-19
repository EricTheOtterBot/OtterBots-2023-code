package frc.robot.autos;

import frc.robot.Constants;
import frc.robot.commands.AutoOpenClaw;
import frc.robot.commands.*;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Lift;


import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class balanceAuto extends SequentialCommandGroup {
    public balanceAuto(Swerve s_Swerve, Claw s_Claw, Lift s_lift){
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
        Trajectory firstTrajectory =
            TrajectoryGenerator.generateTrajectory(

                new Pose2d(0, 0, new Rotation2d(0)),

                List.of(),

                new Pose2d(-1, 0.02, new Rotation2d(0)),
                configB);



        var thetaController =
            new ProfiledPIDController(
                Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);



        SwerveControllerCommand swerveControllerCommand =
            new SwerveControllerCommand(
                firstTrajectory,
                s_Swerve::getPose,
                Constants.Swerve.swerveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                thetaController,
                s_Swerve::setModuleStates,
                s_Swerve);


        Command openclawcommand = new AutoOpenClaw(s_Claw);
        Command stopclawcommand = new AutoStopClaw(s_Claw);
        Command liftcommand = new AutoRaiseLift(s_lift);
        Command extendcommand = new AutoExtend(s_lift);
        Command retractcommand = new AutoRetract(s_lift);
        Command lowerliftcommand = new AutoLowerLift(s_lift);
        Command waitcommand = new WaitCommand(0.5);
        Command waitcommand2 = new WaitCommand(0.5);
        Command waitcommand3 = new WaitCommand(0.5);
        Command closeclawcommand = new AutoOpenClaw(s_Claw);
        Command stopclawcommand2 = new AutoStopClaw(s_Claw);
        Command balance1command = new AutoBalance(s_Swerve);
        Command balance2command = new AutoBalance2(s_Swerve);

        addCommands(
            new InstantCommand(() -> s_Swerve.resetOdometry(firstTrajectory.getInitialPose())), 
            //liftcommand,
            extendcommand,
            openclawcommand, 
            waitcommand, 
            stopclawcommand,
            waitcommand3,
            stopclawcommand2,
            retractcommand, 
            lowerliftcommand,
            swerveControllerCommand
        );
    }
}