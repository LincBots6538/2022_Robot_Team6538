// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Climber;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.kClimber;
import frc.robot.subsystems.Climber;

public class MidToHigh extends CommandBase {
  /** Creates a new ClimbUpSensored. */

  private Timer timer;
  private boolean ifChecked;
  private double velocity;
  
  private Climber sys_climber;
  public MidToHigh(Climber climber) {
    // Use addRequirements() here to declare subsystem dependencies.
    sys_climber = climber;
    addRequirements(sys_climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    sys_climber.resetPosition();

    timer.reset();
    timer.start();
    ifChecked = false;

    velocity = kClimber.CLIMBER_SPEED * kClimber.GEAR_RATIO * 2048;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(ifChecked == false){
      if(timer.get() > .1){
        sys_climber.setClawA(kClimber.CLAW_CLOSED);
        sys_climber.setClawB(kClimber.CLAW_OPEN);
        sys_climber.setMotor(velocity);

        ifChecked = true;
    }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    sys_climber.setClawB(kClimber.CLAW_CLOSED);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return sys_climber.isBarB();
    
  }
}
