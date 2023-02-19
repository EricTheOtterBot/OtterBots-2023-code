package frc.robot.autos;

import frc.robot.Constants;
import frc.robot.commands.AutoOpenClaw;
import frc.robot.commands.balancing.AutoBalance;
import frc.robot.commands.balancing.AutoBalance2;
import frc.robot.commands.balancing.AutoBalance3;
import frc.robot.commands.balancing.AutoBalance4;
import frc.robot.commands.balancing.AutoBalance5;
import frc.robot.commands.balancing.AutoBalance6;
import frc.robot.commands.balancing.AutoBalance7;
import frc.robot.commands.balancing.AutoBalance8;
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
        Command waitcommand = new WaitCommand(0.4);
        Command waitcommand2 = new WaitCommand(0.4);
        Command waitcommand3 = new WaitCommand(0.1);
        Command waitcommand4 = new WaitCommand(0.1);
        Command waitcommand5 = new WaitCommand(0.1);
        Command waitcommand6 = new WaitCommand(0.1);
        Command waitcommand7 = new WaitCommand(0.1);
        Command waitcommand8 = new WaitCommand(0.1);
        Command waitcommand9 = new WaitCommand(0.1);
        Command waitcommand10 = new WaitCommand(0.1);
        Command closeclawcommand = new AutoOpenClaw(s_Claw);
        Command stopclawcommand2 = new AutoStopClaw(s_Claw);
        Command balance1command = new AutoBalance(s_Swerve);
        Command balance2command = new AutoBalance2(s_Swerve);
        Command balance3command = new AutoBalance3(s_Swerve);
        Command balance4command = new AutoBalance4(s_Swerve);
        Command balance5command = new AutoBalance5(s_Swerve);
        Command balance6command = new AutoBalance6(s_Swerve);
        Command balance7command = new AutoBalance7(s_Swerve);
        Command balance8command = new AutoBalance8(s_Swerve);

        addCommands(
            new InstantCommand(() -> s_Swerve.resetOdometry(firstTrajectory.getInitialPose())), 

            extendcommand,
            openclawcommand, 
            waitcommand, 
            stopclawcommand,
            waitcommand3,
            stopclawcommand2,
            retractcommand, 
            lowerliftcommand,
            balance1command,
            waitcommand4,  
            balance2command,
            waitcommand5, 
            balance3command, 
            waitcommand6, 
            balance4command,
            waitcommand7,  
            balance5command, 
            waitcommand8, 
            balance6command, 
            waitcommand9, 
            balance7command, 
            waitcommand10, 
            balance8command
        );
    }
}