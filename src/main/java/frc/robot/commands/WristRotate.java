// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Wrist;

public class WristRotate extends CommandBase {
  // Create some clase scop variables:
  private final Wrist m_Wrist;
  private double m_speed;

  /** Creates a new WristRotate. */
  public WristRotate(double speed, Wrist subsystem) {

    // Copy the constructor parrameters to the clase scop variables:
    m_Wrist = subsystem;
    m_speed = speed;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_Wrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //Here is run only once when the command is started
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // This Method runs every 20ms
    m_Wrist.my_Wrist_Rotate(m_speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //This Method is run when the command is complete (isFinished Returs true)
    // or canceled (a new command is scheduled for this subsystem or a button is useing whileTrue). 

    //So if end we should turn off our motors and let the new command retake control
    m_Wrist.my_Wrist_Rotate(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
