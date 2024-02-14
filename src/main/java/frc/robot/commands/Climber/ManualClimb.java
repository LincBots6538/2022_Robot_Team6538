// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Climber;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.kClimber;
import frc.robot.subsystems.Climber;

public class ManualClimb extends CommandBase {
  /** Creates a new ManualClimb. */
  private Climber sys_climber;

  private int state = 0;
  private DoubleSupplier motor_axis;
  private BooleanSupplier next, prev;
  private boolean change;

  public ManualClimb(Climber climber, DoubleSupplier climb_axis, BooleanSupplier next_state, BooleanSupplier last_state) {
    // Use addRequirements() here to declare subsystem dependencies.
    sys_climber = climber;
    motor_axis = climb_axis;
    prev = next_state;
    next = last_state;
    addRequirements(sys_climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(sys_climber.islocked() == false){
      if (next.getAsBoolean()) {
        state ++; // Next Claw State
        if (state > 7) state = 7;
        change = true;
      } 
      else if (prev.getAsBoolean()){
         state --; // Previous Claw State
         if (state < 0) state = 0;
         change = true;
      }
      
      // Motor Control
      double climb_axis = MathUtil.applyDeadband(motor_axis.getAsDouble(), kClimber.JOYSTICK_DEADBAND);
      //double climb_speed = climb_axis*(200*kClimber.GEAR_RATIO*kClimber.ENCODER_TICKS/600);
      //sys_climber.setMotor(climb_speed);
      sys_climber.setMotor(climb_axis);

      // Claw State Control
      if (change){  // If a new state is commanded
        switch (state) {
          case 0: // Starting Postion
            sys_climber.setClawA(kClimber.CLAW_PASS);
            sys_climber.setClawB(kClimber.CLAW_PASS);
            break;
          case 1: // Align on Mid Bar
            sys_climber.setClawA(kClimber.CLAW_OPEN);
            sys_climber.setClawB(kClimber.CLAW_OPEN);
            break;
          case 2: // Grip Mid Bar
            sys_climber.setClawA(kClimber.CLAW_CLOSED);
            break;
          case 3: // Grip High Bar
            sys_climber.setClawB(kClimber.CLAW_CLOSED);
            break;
          case 4: // Release Mid Bar
            sys_climber.setClawA(kClimber.CLAW_PASS);
            break;
          case 5: // Prep Hook for Traverse Bar
            sys_climber.setClawA(kClimber.CLAW_OPEN);
            break;
          case 6: // Grip Traverse Bar
            sys_climber.setClawA(kClimber.CLAW_CLOSED);
            break;
          case 7: // Release High Bar
            sys_climber.setClawB(kClimber.CLAW_PASS);
            break;
        }
        change = false;
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
