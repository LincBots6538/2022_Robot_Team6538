// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.LimeLight;
import frc.robot.RobotContainer;
import frc.robot.Constants.kDRIVE;
import frc.robot.subsystems.SpeedyPuffJunior;

public class TrackingBall extends CommandBase {

  private final SpeedyPuffJunior m_speedypuffjunior;
  /** Creates a new TrackingBall. */
  private double ballx = 0;

  public TrackingBall(SpeedyPuffJunior subsystem){
  

    m_speedypuffjunior = subsystem;
    addRequirements(m_speedypuffjunior);
    // Use addRequirements() here to declare subsystem dependencies.
}

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    
   double Speed = MathUtil.applyDeadband(RobotContainer.GetDriveAxis(kDRIVE.SPEED_AXIS), kDRIVE.JOYSTICK_DEADBAND);
   ballx = LimeLight.getTargetx();

   /* I was here*/
   if (LimeLight.getTargetArea() >= 3.5){
   Speed = 0;
  }
    if ( ballx > 3){
      m_speedypuffjunior.arcadeDrive(Speed , .15*(Math.min(Math.abs(ballx)/15,1)));
    }
    else if ( ballx < -3){
      m_speedypuffjunior.arcadeDrive(Speed , -.15*(Math.min(Math.abs(ballx)/15,1)));
    }
    else {
      m_speedypuffjunior.arcadeDrive(Speed , 0);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
