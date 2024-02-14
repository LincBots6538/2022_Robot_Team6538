// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.kDRIVE;
import frc.robot.subsystems.SpeedyPuffJunior;

public class DriveFoward extends CommandBase {
  /** Creates a new FaceAngle. */
  private final SpeedyPuffJunior m_speedypuffjunior;
  private double setDis, ltarget, rtarget;

  public DriveFoward(SpeedyPuffJunior subsystem, double dis) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_speedypuffjunior = subsystem;
    addRequirements(subsystem);

    setDis = dis;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_speedypuffjunior.setPIDSlot(2);
    //m_speedypuffjunior.resetPosition();

    double distance = ((kDRIVE.GEAR_RATIO * 2048 * 12) / (kDRIVE.WHEEL_DIAMETER * Math.PI)) * setDis; // Units in feet
    
    ltarget = m_speedypuffjunior.getPostion(true) + distance;
    rtarget = m_speedypuffjunior.getPostion(false) - distance;


  }

  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_speedypuffjunior.positionDrive(rtarget, ltarget);
    SmartDashboard.putNumber("LeftTarget", ltarget);
    SmartDashboard.putNumber("RightTarget", rtarget);
    SmartDashboard.putNumber("LeftActuals", m_speedypuffjunior.getPostion(true));
    SmartDashboard.putNumber("RightActuals",m_speedypuffjunior.getPostion(false));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(m_speedypuffjunior.getPostion(true) > (ltarget* 0.9) && m_speedypuffjunior.getPostion(true) < (ltarget * 1.1)){
      if(m_speedypuffjunior.getMotorOutput() == 0){
        return true;
      }
    }
    else{
      return false;
    }
    return false;
  }
}
