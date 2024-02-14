// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.kElevator;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ubIntake;

public class Shoot extends CommandBase {
  /** Creates a new ShootLG. */
  private Shooter s_Shooter;
  private Elevator s_Elevator;
  private ubIntake s_ubintake;

  private Timer delayTimer = new Timer();
  private double timeout = 3;

  public Shoot(Shooter shooter, Elevator elevator, ubIntake ubintake) {
    // Use addRequirements() here to declare subsystem dependencies.
    s_Shooter = shooter;
    s_Elevator = elevator;
    s_ubintake = ubintake;
    addRequirements(s_Elevator, s_Shooter, s_ubintake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    s_Elevator.ready();
    s_Shooter.spinUP();
    timeout =3;
    delayTimer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Fire Ball when wheels are at speed and elevator is lowered
    if (s_Shooter.atSpeed() && !s_Elevator.isUp()) {
      s_Elevator.load();
      delayTimer.reset();
      timeout = kElevator.DELAY;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_Elevator.ready();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (delayTimer.get() > timeout) {
      return true;
    } else {
      return false;
    }
  }
}
