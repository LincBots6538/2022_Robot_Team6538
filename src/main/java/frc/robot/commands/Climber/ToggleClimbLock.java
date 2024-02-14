// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Climber;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Climber;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ToggleClimbLock extends InstantCommand {
  private Climber s_climber;
  private boolean lockState;

  public ToggleClimbLock(Climber climber) {
    // Use addRequirements() here to declare subsystem dependencies.
    s_climber = climber;

    addRequirements(s_climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    lockState = s_climber.islocked();
    if (lockState){
      s_climber.lock(false);
    }
    else{
      s_climber.lock(true);
    }
    
  }
}
