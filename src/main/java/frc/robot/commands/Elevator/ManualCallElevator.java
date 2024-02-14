// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator;

public class ManualCallElevator extends CommandBase {
  /** Creates a new ManualCallElevator. */

  public final Elevator s_elevator;

  public ManualCallElevator(Elevator subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    s_elevator = subsystem;
    addRequirements(s_elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    /*if(s_elevator.isUp() == true){
      s_elevator.ready();
    }
    
    if (s_elevator.isUp() == false){
      s_elevator.load();
    }
    */
    
      s_elevator.load();
    
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_elevator.ready();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

