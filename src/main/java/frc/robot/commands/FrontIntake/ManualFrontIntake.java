// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.FrontIntake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.FrontIntake;
import frc.robot.subsystems.ubIntake;


public class ManualFrontIntake extends CommandBase {
  /** Creates a new ManualActuateFrontIntake. */

  private FrontIntake s_frontIntake;
  private ubIntake s_ubIntake;

  public ManualFrontIntake(FrontIntake subsystem, ubIntake subsystem1) {
    // Use addRequirements() here to declare subsystem dependencies.
    s_frontIntake = subsystem;
    s_ubIntake = subsystem1;
    addRequirements(s_frontIntake, s_ubIntake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    s_frontIntake.setDown();
    s_ubIntake.setIntaking();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_frontIntake.setUp();
    s_ubIntake.setStored();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
