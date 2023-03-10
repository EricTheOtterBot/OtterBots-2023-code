package frc.robot.commands;

import frc.robot.subsystems.Lift;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class EricAutoRetractAndLower extends CommandBase {
    Lift s_ClawLift;
    double extensionVal;
    double liftVal;
    boolean wantLimitSwitch;
    boolean wantUp;
    boolean wantOut;
    double time;
    double timeS;
    Timer m_timer;

    
    public EricAutoRetractAndLower(Lift s_ClawLift, double extensionVal, double liftVal, boolean wantLimitSwitch, boolean wantUp, boolean wantOut, double time, Timer m_timer) {
        this.s_ClawLift = s_ClawLift;
        this.extensionVal = extensionVal;
        this.liftVal = liftVal;
        this.wantLimitSwitch = wantLimitSwitch;
        this.wantUp = wantUp;
        this.wantOut = wantOut;
        this.timeS = 0.0;
        this.time = time;
        this.m_timer = m_timer;
        addRequirements(s_ClawLift);
    }
    
    @Override
    public void initialize() {
        timeS = m_timer.get();
    }


    @Override
    public void execute() {
        if(wantLimitSwitch) {
            if(wantUp) {
                if(s_ClawLift.get_top_limit()) {
                    s_ClawLift.auto_raise_set_speed(liftVal);
                } else {
                    s_ClawLift.auto_stop_raise();
                }
            } else {
                if(!s_ClawLift.get_bottom_limit()) {
                    s_ClawLift.auto_raise_set_speed(liftVal);
                } else {
                    s_ClawLift.auto_stop_raise();
                }
            }
            if(wantOut) {
                if(!s_ClawLift.get_front_limit()) {
                    s_ClawLift.auto_extend_set_speed(extensionVal);
                } else {
                    s_ClawLift.auto_stop_extend();
                }
            } else {
                if(s_ClawLift.get_back_limit()) {
                    s_ClawLift.auto_extend_set_speed(extensionVal);
                } else {
                    s_ClawLift.auto_stop_extend();
                }
            }
               
        } else {
            s_ClawLift.auto_raise_set_speed(liftVal);
            s_ClawLift.auto_extend_set_speed(extensionVal);
        }
    }
        
    @Override
    public boolean isFinished() {
        if(wantLimitSwitch) {
            if(!wantUp && !wantOut) {
                if(!s_ClawLift.get_back_limit() && s_ClawLift.get_bottom_limit()) {
                    return true;
                }
            return false;
            } else {
                if(!s_ClawLift.get_top_limit() && s_ClawLift.get_front_limit()) {
                    return true;
                } else {
                    return false;
                }
            }
        } else {
            if(m_timer.get() - timeS >= time) {
                return true;
            }
            else {
                return false;
            }
        }
    }    

    @Override
    public void end(boolean Interrupted) {
        s_ClawLift.auto_stop_extend();
        s_ClawLift.auto_stop_raise();
    }


}
