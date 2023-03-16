package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.oi.LimeLight;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    SendableChooser<Command> auto_chooser = new SendableChooser<>();

    /* Controllers */
    private final Joystick driver = new Joystick(2);
    private final Joystick operator = new Joystick(3);

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;
    private final int speedControl = XboxController.Axis.kRightTrigger.value;
    
    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    private final JoystickButton targetLock = new JoystickButton(driver, XboxController.Button.kA.value);
    
    /* Operator Controls */
    private final int extensionAxis = XboxController.Axis.kLeftY.value;
    private final int liftAxis = XboxController.Axis.kRightY.value;

    /* Operator Buttons */
    private final JoystickButton rotateClawF_PB = new JoystickButton(operator, XboxController.Button.kB.value);
    private final JoystickButton rotateClawR_PB = new JoystickButton(operator, XboxController.Button.kA.value);
    private final JoystickButton closeClaw_PB = new JoystickButton(operator, XboxController.Button.kLeftBumper.value);
    private final JoystickButton openClaw_PB = new JoystickButton(operator, XboxController.Button.kRightBumper.value);

    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    private final Lift s_Lift = new Lift();
    private final Claw s_Claw = new Claw();
    private final Wrist s_Wrist = new Wrist();

    private final LimeLight a_limelight = new LimeLight();
    private final LimelightVision a_limelightvision = new LimelightVision();

    public final Timer m_timer = new Timer();
    public double gyroOffset = 0.0;

    /* Autos */
    private final Command auto1 = new AutoOnePiece(s_Swerve, s_Lift, s_Claw, m_timer);
    private final Command auto4 = new AutoBalance(s_Swerve, s_Claw, s_Lift, m_timer);
    private final Command auto3 = new AutoOnePieceNoBacking(s_Swerve, s_Lift, s_Claw, m_timer);
    private final Command auto6 = new AutoHighCubeL(a_limelightvision, s_Swerve, s_Claw, s_Lift, m_timer);
    private final Command auto7 = new AutoHighCubeR(a_limelightvision, s_Swerve, s_Claw, s_Lift, m_timer);

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {     
        
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -driver.getRawAxis(translationAxis)/1.3, 
                () -> -driver.getRawAxis(strafeAxis)/1.3, 
                () -> -driver.getRawAxis(rotationAxis)/2/9*6.75, 
                () -> robotCentric.getAsBoolean(), 
                () -> targetLock.getAsBoolean(),
                () -> a_limelight.getdegRotationToTarget(),
                () -> driver.getRawAxis(speedControl),
                () -> gyroOffset
            )
        );

        s_Lift.setDefaultCommand(
            new TeleopLift(
                s_Lift,
                () -> operator.getRawAxis(extensionAxis), 
                () -> -operator.getRawAxis(liftAxis)
            )
        );
        m_timer.start();
        /* s_Claw.setDefaultCommand(
            new TeleopClaw(
                s_Claw,
                () -> rotateClawF.getAsBoolean(), 
                () -> rotateClawR.getAsBoolean(), 
                () -> closeClaw.getAsBoolean(), 
                () -> openClaw.getAsBoolean()
            )
        ); */


        // Configure the button bindings
        configureButtonBindings();

        auto_chooser.setDefaultOption("One Piece No Backing", auto3);
        auto_chooser.addOption("Auto Balance", auto4);
        auto_chooser.addOption("One Piece", auto1);
        auto_chooser.addOption("High Cube L", auto6);
        auto_chooser.addOption("High Cube R", auto7);
        SmartDashboard.putData(auto_chooser);
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
        zeroGyro.onTrue(new InstantCommand(() -> this.zeroGyro()));
        // rotateClaw.onTrue(new InstantCommand(() -> CL.rotate_claw()));
        rotateClawR_PB.whileTrue(new WristRotate(-1, s_Wrist));
        rotateClawF_PB.whileTrue(new WristRotate(1, s_Wrist));
        //rotateClaw.onFalse(new InstantCommand(() -> CL.stop_claw()));
        // rotateClawR.onFalse(new InstantCommand(() -> CL.stop_claw()));
        closeClaw_PB.whileTrue(new ClawRun(0.8, s_Claw));
        openClaw_PB.whileTrue(new ClawRun(-0.8, s_Claw));
        // closeClaw.onFalse(new InstantCommand(() -> CL.stop_claw_g()));
        // openClaw.onFalse(new InstantCommand(() -> CL.stop_claw_g()));
    }

    public void zeroGyro() {
        s_Swerve.zeroGyro();
        gyroOffset = 0.0;
    }

    public void setPipeline(int pipeline) {
        a_limelightvision.setPipeline(pipeline);
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        // return new testAutoClawLift(s_Swerve, s_Lift);
        return auto_chooser.getSelected();
    }
}
