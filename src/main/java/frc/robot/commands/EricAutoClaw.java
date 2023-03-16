package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.Swerve;



import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class EricAutoClaw extends CommandBase {    
    private Swerve s_Swerve;
    private double speedVal;
    private boolean wantGrip;
    private Timer m_timer;
    private double time;
    private double timeS;
    private Claw s_Claw; 

    public EricAutoClaw(Claw s_Claw, double speedVal, boolean wantGrip, double time, Timer m_timer) {
        this.s_Claw = s_Claw;
        this.speedVal = speedVal;
        this.wantGrip = wantGrip;
        this.m_timer = m_timer;
        this.time = time;
        this.timeS = 0.0;
        addRequirements(s_Claw);
    } 

    @Override
    public void initialize() {
        timeS = m_timer.get();
    }

    @Override
    public void execute() {
        if(wantGrip)  {
            s_Claw.auto_grip_set_speed(speedVal);
        } else {
            //This will be handeled in other ways
            //s_Claw.auto_turn_set_speed(speedVal);
        }
    }

    @Override
    public boolean isFinished() {
        if(m_timer.get() - timeS >= time) {
            return true;
        }
        else {
            return false;
        }

    }

}
 