// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ubIntake;

public class ManualIntake extends CommandBase {
  /** Creates a new ManualIntake. */
  private ubIntake s_ubIntake;
  public ManualIntake(ubIntake ubintake) {
    // Use addRequirements() here to declare subsystem dependencies.
    s_ubIntake = ubintake;
    addRequirements(s_ubIntake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    s_ubIntake.setIntaking();
  }
    
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_ubIntake.setStored();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    /*if(s_ubIntake.isBallStored() && !s_ubIntake.isBusy()){
      return true;
    }
    return false;
  }  
  */
  return false;
  }
}
