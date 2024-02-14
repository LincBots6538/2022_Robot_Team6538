// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Climber;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.kClimber;
import frc.robot.subsystems.Climber;

public class ReleaseHigh extends CommandBase {
  /** Creates a new ReleaseHigh. */
  private Climber sys_climber;
  private Timer timer = new Timer();
  private int task = 1;
  private double time;

  public ReleaseHigh(Climber climber) {
    // Use addRequirements() here to declare subsystem dependencies.
    sys_climber = climber;
    addRequirements(sys_climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.start();
    sys_climber.setMotor(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    time = timer.get();
    if (time > 0.3 && task == 1){
      sys_climber.setMotor(-kClimber.CLIMBER_SPEED/4);  // Anti Swing back drive
      task = 2;
    }
    else if (time > 0.5 && task == 2){
      sys_climber.setClawB(kClimber.CLAW_PASS);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (time > 0.7){
      return true;
    }
    return false;
  }
}
