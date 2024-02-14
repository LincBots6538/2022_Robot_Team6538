// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ChangeSpin extends InstantCommand {
  private Shooter s_shooter;
  private double dspin, current;
  public ChangeSpin(Shooter shooter, double deltaSpin) {
    // Use addRequirements() here to declare subsystem dependencies.
    s_shooter = shooter;
    dspin = deltaSpin;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    current = s_shooter.getSpin();
    s_shooter.setSpin(current + dspin);
  }
}
