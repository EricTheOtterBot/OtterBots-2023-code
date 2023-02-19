    package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.Constants.ClawLiftConstants;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.AnalogTrigger;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;


public class Lift extends SubsystemBase {
    private VictorSPX m_extender;
    private TalonSRX m_lift;
    private DigitalInput s_limitswitchtop;
    private DigitalInput s_limitswitchback;
    private DigitalInput s_limitswitchbottom;
    private DigitalInput s_limitswitchfront;


    public Lift() {
        m_extender = new VictorSPX(ClawLiftConstants.can_id_extender);
        m_lift = new TalonSRX(ClawLiftConstants.can_id_lift);
        // a_claw_rotate = new AnalogTrigger(ClawLiftConstants.ai_claw_rotate);
        // s_claw_rotate = new Counter(a_claw_rotate);
        s_limitswitchtop = new DigitalInput(ClawLiftConstants.dio_top_limit_switch);
        s_limitswitchback = new DigitalInput(ClawLiftConstants.dio_back_limit_switch);
        s_limitswitchbottom = new DigitalInput(ClawLiftConstants.dio_bottom_limit_switch);
        s_limitswitchfront = new DigitalInput(ClawLiftConstants.dio_front_limit_switch);
    }


    public void default_control(double extensionVal, double liftVal) {        
        /* Lift */
        if(s_limitswitchtop.get() == true && liftVal > 0) {
            liftVal = 0;
        } 
        if(s_limitswitchbottom.get() == false && liftVal < 0) {
            liftVal = 0;
        }
        
        m_lift.set(ControlMode.PercentOutput, liftVal);

        /* Extender */
        if(s_limitswitchback.get() == true && extensionVal > 0) {
            extensionVal = 0;
        }
        if(s_limitswitchfront.get() == false && extensionVal < 0) {
            extensionVal = 0;
        }
        
        m_extender.set(ControlMode.PercentOutput, extensionVal);
       
    }

    public void auto_raise() {
        m_lift.set(ControlMode.PercentOutput, 0.5);
    }
    public void auto_stop_raise() {
        m_lift.set(ControlMode.PercentOutput, 0);
    }
    public boolean get_top_limit() {
        return s_limitswitchtop.get();
    }
    public void auto_lower() {
        m_lift.set(ControlMode.PercentOutput, -1);
    }
    public boolean get_bottom_limit() {
        return !s_limitswitchbottom.get();
    }



    public void auto_extend() {
        m_extender.set(ControlMode.PercentOutput, -1);
    }
    public void auto_stop_extend() {
        m_extender.set(ControlMode.PercentOutput, 0);
    }
    public boolean get_front_limit() {
        return !s_limitswitchfront.get();
    }
    public void auto_retract() {
        m_extender.set(ControlMode.PercentOutput, 1);
    }
    public boolean get_back_limit() {
        return s_limitswitchback.get();
    }

    @Override
    public void periodic(){
        SmartDashboard.putBoolean("FrontLimit", s_limitswitchfront.get());
        SmartDashboard.putBoolean("BackLimit", s_limitswitchback.get());
        SmartDashboard.putBoolean("TopLimit", s_limitswitchtop.get());
        SmartDashboard.putBoolean("BottomLimit", s_limitswitchbottom.get());

    }
}
