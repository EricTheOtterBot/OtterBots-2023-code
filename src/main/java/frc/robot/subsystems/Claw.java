package frc.robot.subsystems;

import frc.robot.Constants.ClawLiftConstants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.ControlMode;


public class Claw extends SubsystemBase {
   
    private VictorSPX m_claw_grip;


    public Claw() {
       
        m_claw_grip = new VictorSPX(ClawLiftConstants.can_id_claw_grip);
    }


/*     public void default_control(boolean clawF, boolean clawR, boolean clawClose, boolean clawOpen) {        
        //double rotate_speed = 0.0;
        double grip_speed = 0.0;

        if(clawF == true) {
            rotate_speed += 1;
        }
        if(clawR == true) {
            rotate_speed -= 1;
        } 

        if(clawClose == true) {
            grip_speed += 0.8;
        }
        if(clawOpen == true) {
            grip_speed -= 0.8;
        }

        
        m_claw_grip.set(ControlMode.PercentOutput, grip_speed);

    } */

    /**
     * Run the Gipper
     * @param grip_speed -1 to 1
     */
    public void my_GipperRun(double grip_speed){
        m_claw_grip.set(ControlMode.PercentOutput, grip_speed);
    }

    /**
     * I did not toucht these but if we rewote the commands all of these should be
     * covered by the above method
    */

    public void auto_open() {
        m_claw_grip.set(ControlMode.PercentOutput, 1);
    }
    public void auto_close() {
        m_claw_grip.set(ControlMode.PercentOutput, -1);
    }
    public void auto_stop() {
        m_claw_grip.set(ControlMode.PercentOutput, 0);
    }
    public void auto_grip_set_speed(double speedee) {
        m_claw_grip.set(ControlMode.PercentOutput, speedee);
    }
    
}
