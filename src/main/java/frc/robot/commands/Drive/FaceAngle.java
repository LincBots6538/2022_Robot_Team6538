// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.kDRIVE;
import frc.robot.subsystems.SpeedyPuffJunior;

public class FaceAngle extends CommandBase {
  /** Creates a new FaceAngle. */
  private final SpeedyPuffJunior m_speedypuffjunior;
  private double setDeg, ltarget, rtarget;
  private Timer timer;
  private boolean ifChecked;

  public FaceAngle(SpeedyPuffJunior subsystem, double deg) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_speedypuffjunior = subsystem;
    addRequirements(subsystem);
    timer = new Timer();
    setDeg = deg;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    m_speedypuffjunior.setPIDSlot(1);
    m_speedypuffjunior.resetPosition();
    timer.start();

    ifChecked = false;

    ltarget = 0;
    rtarget = 0;

    SmartDashboard.putString("command", "Face Angle not finished");
  }

  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // If the setpoint has not been set then set the setpoint 
    // If the setpoint HAS been set then dont set the setpoint again
    if(ifChecked == false){
      if(timer.get() > .1){
        double distance = (kDRIVE.WIDTH/kDRIVE.WHEEL_DIAMETER) * (setDeg/360) * kDRIVE.GEAR_RATIO * 2048;  // Arclength distance of twist
    
        ltarget = m_speedypuffjunior.getPostion(true) + distance;
        rtarget = m_speedypuffjunior.getPostion(false) + distance;

        ifChecked = true; 
      }
    }
    // Drive to the setpoint
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
    // Checks if setpoint has been reached
    // If checkpoint has been reached, then finish command
    if(Math.abs(m_speedypuffjunior.getPostion(true)) > (Math.abs(ltarget) - 100) && Math.abs(m_speedypuffjunior.getPostion(true)) < (Math.abs(ltarget) + 100)){
      SmartDashboard.putString("command", "Face Angle Finished");
      if(m_speedypuffjunior.getMotorOutput() == 0){
        return true;
      }
    }
    return false;
  }
}
