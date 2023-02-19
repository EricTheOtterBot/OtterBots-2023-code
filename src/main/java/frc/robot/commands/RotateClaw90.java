// package frc.robot.commands;

// import frc.robot.Constants.ClawLiftConstants;
// import frc.robot.subsystems.ClawLift;

// import java.util.function.BooleanSupplier;
// import java.util.function.DoubleSupplier;

// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.wpilibj2.command.CommandBase;

// public class RotateClaw90 extends CommandBase {
//     ClawLift s_ClawLift;
//     BooleanSupplier rotateSup;
//     double counts_per_90;
//     boolean start_state;  // true = vertical, false = horizontal
//     int start_count;
//     double end_count;  // double type because we need to add counts_per_90 to start_count

//     public RotateClaw90(ClawLift s_ClawLift, BooleanSupplier rotateSup) {
//         this.counts_per_90 = ClawLiftConstants.counts_per_rev / 4;
//         this.s_ClawLift = s_ClawLift;
//         this.rotateSup = rotateSup;
//     }

//     @Override
//     public void initialize() {
//         start_count = s_ClawLift.get_rotation_counter();
//         start_state = s_ClawLift.get_claw_rotation_state();

//         if(start_state == false) {
//             this.end_count = this.start_count + this.counts_per_90;
//         } else if(start_state == true) {
//             this.end_count = this.start_count - this.counts_per_90;
//         }
//     }

//     @Override
//     public void execute() {
//         if(start_state == false) {
//             s_ClawLift.rotate_claw_cw();
//         } else if(start_state == true) {
//             s_ClawLift.rotate_claw_ccw();
//         }
//     }

//     @Override
//     public boolean isFinished() {
//         /* Return true when the claw has reached the desired rotational position */
//         // TODO: Is there an issue with negative numbers, etc?
//         boolean finished = false;
//         if(start_state == false) {
//             finished = (s_ClawLift.get_rotation_counter() > end_count);
//         } else if(start_state == true) {
//             finished = (s_ClawLift.get_rotation_counter() < end_count);
//         }

//         return finished;
//     }

//     @Override
//     public void end(boolean interrupted) {
//         // Stop rotation, and set the state to the opposite of what it was
//         s_ClawLift.stop_claw_rotation();
//         s_ClawLift.set_claw_rotation_state(!start_state);
//     }

// }
