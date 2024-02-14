// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ubIntake;

public class ManualAdvance extends CommandBase {
  /** Creates a new ManualAdvance. */
  private ubIntake s_ubIntake;


  public ManualAdvance(ubIntake ubintake) {
    // Use addRequirements() here to declare subsystem dependencies.
    s_ubIntake = ubintake;
    addRequirements(s_ubIntake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    s_ubIntake.setAdvance();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (!interrupted) {
      if (s_ubIntake.isBallStored() == true)  
        s_ubIntake.setSensorError(true);
    }
    else s_ubIntake.setStored();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !s_ubIntake.isBusy();
  }
}
