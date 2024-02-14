// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import java.util.function.DoubleSupplier;

import org.opencv.core.Mat;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.kDRIVE;
import frc.robot.subsystems.SpeedyPuffJunior;



public class DriveTeleop extends CommandBase {
  /** Creates a new DriveTeleop. */
  private final SpeedyPuffJunior m_speedypuffjunior;
  private final DoubleSupplier m_speedAxis;
  private final DoubleSupplier m_turnAxis;
  private double Speed;
  private double Turn; 

  public DriveTeleop(SpeedyPuffJunior subsystem, DoubleSupplier Dspeed, DoubleSupplier Dturn) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_speedypuffjunior = subsystem;
    m_speedAxis = Dspeed;
    m_turnAxis = Dturn;
    addRequirements(m_speedypuffjunior);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    m_speedypuffjunior.setPIDSlot(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Apply Deadband to Drive Inputs
    

    //Speed = MathUtil.applyDeadband(Math.signum(m_speedAxis.getAsDouble())*Math.pow(m_speedAxis.getAsDouble(),2), kDRIVE.JOYSTICK_DEADBAND);
    //Turn = MathUtil.applyDeadband(Math.signum(m_turnAxis.getAsDouble())*Math.pow(m_turnAxis.getAsDouble(),2), kDRIVE.JOYSTICK_DEADBAND);

    //double tempturn = m_turnAxis.getAsDouble();

    Speed = MathUtil.applyDeadband(m_speedAxis.getAsDouble(), kDRIVE.JOYSTICK_DEADBAND);
    Turn = MathUtil.applyDeadband(m_turnAxis.getAsDouble(), kDRIVE.JOYSTICK_DEADBAND);
    //Turn = MathUtil.applyDeadband(Math.abs(tempturn)*tempturn, kDRIVE.JOYSTICK_DEADBAND);
    // Call Arcade Drive Function
    m_speedypuffjunior.arcadeDrive(Speed, Turn);
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
