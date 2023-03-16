// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClawLiftConstants;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.ControlMode;

public class Wrist extends SubsystemBase {

  private VictorSPX m_claw_rotate;

  /** Creates a new Wrist. */
  public Wrist() {
    m_claw_rotate = new VictorSPX(ClawLiftConstants.can_id_claw_rotate);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void my_Wrist_Rotate(double rotate_speed){
    m_claw_rotate.set(ControlMode.PercentOutput, rotate_speed);
  }

  //Not Require now
/*   public void auto_turn_set_speed(double speedeee) {
    m_claw_rotate.set(ControlMode.PercentOutput, speedeee);
  } */
}
