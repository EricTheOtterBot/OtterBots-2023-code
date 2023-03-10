package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.Swerve;



import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class EricAutoLift extends CommandBase {    
    private Swerve s_Swerve;
    private double liftVal;
    private boolean wantUp;
    private boolean wantLimitSwitch;
    private Timer m_timer;
    private double time;
    private double timeS;
    private Lift s_ClawLift; 

    public EricAutoLift(Lift s_ClawLift, double liftVal, boolean wantUp, boolean wantLimitSwitch, double time, Timer m_timer) {
        this.s_ClawLift = s_ClawLift;
        this.liftVal = liftVal;
        this.wantUp = wantUp;
        this.wantLimitSwitch = wantLimitSwitch;
        this.m_timer = m_timer;
        this.time = time;
        this.timeS = 0.0;
        addRequirements(s_ClawLift);
    } 

    @Override
    public void initialize() {
        timeS = m_timer.get();
    }

    @Override
    public void execute() {
        s_ClawLift.auto_raise_set_speed(liftVal);   
    }

    @Override
    public boolean isFinished() {
        if(wantLimitSwitch) {
            if(wantUp) {
                if(!s_ClawLift.get_top_limit()) {
                    return true;
                } else {
                    return false;
                }
            } else {
                if(!s_ClawLift.get_bottom_limit()) {
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
        s_ClawLift.auto_stop_raise();
    }

}
 