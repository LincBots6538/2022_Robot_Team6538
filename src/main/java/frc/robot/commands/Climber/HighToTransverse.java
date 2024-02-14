// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.kClimber;
import frc.robot.subsystems.Climber;

public class HighToTransverse extends CommandBase {
  /** Creates a new HighToTransverse. */

  private double velocity;
  
  private Climber sys_climber;
  public HighToTransverse(Climber climber) {
    // Use addRequirements() here to declare subsystem dependencies.
    sys_climber = climber;
    addRequirements(sys_climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    velocity = kClimber.CLIMBER_SPEED * kClimber.GEAR_RATIO * 2048;

    sys_climber.setMotor(velocity);
    sys_climber.setClawA(kClimber.CLAW_OPEN);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    sys_climber.setClawA(kClimber.CLAW_CLOSED);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return sys_climber.isBarA();
  }
}
