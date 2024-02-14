// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ubIntake;

public class SearchBall extends CommandBase {
  /** Creates a new SearchBall. */

  private ubIntake s_ubIntake;
  private BooleanSupplier elevatorlock;

  public SearchBall(ubIntake subsytem, BooleanSupplier elevatorup) {
    // Use addRequirements() here to declare subsystem dependencies.
    s_ubIntake = subsytem;
    elevatorlock = elevatorup;
    addRequirements(s_ubIntake);
  } 

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    s_ubIntake.setSearching();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  // Stay in searching until a ball is seen. If offensive intake, if defensive reject
    if(!s_ubIntake.isBusy()){       // is the intake busy
      if (!s_ubIntake.isBallStored()){    // is there a ball stored already
          if(s_ubIntake.isOffensive()){   
            s_ubIntake.setIntaking();
          }
          else if (s_ubIntake.isDefensive()){  
            s_ubIntake.setPassUnder();
          }
          else {    
            if(s_ubIntake.getCurrentState() != 5) s_ubIntake.setSearching();
          }
      }
      else if (s_ubIntake.getCurrentState() == 0 && !elevatorlock.getAsBoolean() && !s_ubIntake.isBallLoaded()) {   // Advance if space in elevator
        s_ubIntake.setAdvance();
      }
      else {
        if(s_ubIntake.getCurrentState() != 0) s_ubIntake.setStored();
      }
      
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;

  }
}
