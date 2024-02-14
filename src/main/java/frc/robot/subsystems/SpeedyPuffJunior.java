// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.kDASHBOARD;
import frc.robot.Constants.kDRIVE;

public class SpeedyPuffJunior extends SubsystemBase {
  /** Creates a new SpeedyPuffJunior. */

  // Speed Controller Objects
  private TalonFX m_frontleft_drive = new TalonFX(kDRIVE.FRONT_LEFT_CANID);
  private TalonFX m_frontright_drive = new TalonFX(kDRIVE.FRONT_RIGHT_CANID);
  private TalonFX m_backleft_drive = new TalonFX(kDRIVE.REAR_LEFT_CANID);
  private TalonFX m_backright_drive = new TalonFX(kDRIVE.REAR_RIGHT_CANID);

  // Drive Control Varibles
  private double rdrive_sp_act, ldrive_sp_act, rdrive_sp_cmd, ldrive_sp_cmd, chassis_sp;

  private int loop_time;

  public SpeedyPuffJunior() {
    //sets the encoder position to 0
    m_frontleft_drive.setSelectedSensorPosition(0);
    m_frontright_drive.setSelectedSensorPosition(0);
    
    // Set rear motors to follow front drive motors
    m_backleft_drive.follow(m_frontleft_drive);
    m_backright_drive.follow(m_frontright_drive);

    // Set motor controllers to coast
    m_frontleft_drive.setNeutralMode(NeutralMode.Coast);
    m_backleft_drive.setNeutralMode(NeutralMode.Coast);
    m_frontright_drive.setNeutralMode(NeutralMode.Coast);
    m_backright_drive.setNeutralMode(NeutralMode.Coast);

    //sets the phase of the sensor
    m_frontleft_drive.setSensorPhase(true);
    m_frontright_drive.setSensorPhase(true);

    // Ramp Rates for Percent Output Control Mode:
    m_frontleft_drive.configOpenloopRamp(kDRIVE.RAMP_RATE);
    m_frontright_drive.configOpenloopRamp(kDRIVE.RAMP_RATE);

    

    // Ramp Rates for Velocity & Position Control Modes:
    m_frontleft_drive.configClosedloopRamp(kDRIVE.RAMP_RATE);
    m_frontright_drive.configClosedloopRamp(kDRIVE.RAMP_RATE);

    // Sets PID and peak output values for slot 0
    m_frontleft_drive.config_kP(0, kDRIVE.KP_value_s0);
    m_frontleft_drive.config_kF(0, kDRIVE.KF_value_s0);
    m_frontright_drive.config_kP(0, kDRIVE.KP_value_s0);
    m_frontright_drive.config_kF(0, kDRIVE.KF_value_s0);
    m_frontleft_drive.configAllowableClosedloopError(0, kDRIVE.PID_Error_s0);
    m_frontright_drive.configAllowableClosedloopError(0, kDRIVE.PID_Error_s0);

    m_frontleft_drive.configClosedLoopPeakOutput(0, 1);
    m_frontright_drive.configClosedLoopPeakOutput(0, 1);


    // Sets PID and peak output values for slot 1
    m_frontleft_drive.config_kP(1, kDRIVE.KP_value_s1);
    m_frontleft_drive.config_kF(1, kDRIVE.KF_value_s1);
    m_frontright_drive.config_kP(1, kDRIVE.KP_value_s1);
    m_frontright_drive.config_kF(1, kDRIVE.KF_value_s1);
    m_frontleft_drive.configAllowableClosedloopError(1, kDRIVE.PID_Error_s1);
    m_frontright_drive.configAllowableClosedloopError(1, kDRIVE.PID_Error_s1);

    m_frontleft_drive.configClosedLoopPeakOutput(1, .25);
    m_frontright_drive.configClosedLoopPeakOutput(1, .25);

    // Sets PID and peak output values for slot 2
    m_frontleft_drive.config_kP(2, kDRIVE.KP_value_s2);
    m_frontleft_drive.config_kF(2, kDRIVE.KF_value_s2);
    m_frontright_drive.config_kP(2, kDRIVE.KP_value_s2);
    m_frontright_drive.config_kF(2, kDRIVE.KF_value_s2);
    m_frontleft_drive.configAllowableClosedloopError(2, kDRIVE.PID_Error_s2);
    m_frontright_drive.configAllowableClosedloopError(2, kDRIVE.PID_Error_s2);

    m_frontleft_drive.configClosedLoopPeakOutput(2, .5);
    m_frontright_drive.configClosedLoopPeakOutput(2, .5);

    

  }
  // Resets the encoder position
  public void resetPosition(){

    m_frontleft_drive.setSelectedSensorPosition(0, 0, 100);
    m_frontright_drive.setSelectedSensorPosition(0, 0, 100);
    }

  // Selects a PID slot
  public void setPIDSlot(int slot){

    m_frontleft_drive.selectProfileSlot(slot, 0);
    m_frontright_drive.selectProfileSlot(slot, 0);

  }


  public void arcadeDrive(double speed, double turn){
    //Calculate Chassis Speed
    //rdrive_sp_act = m_frontleft_drive.getSelectedSensorVelocity()/kDRIVE.MAX_VELOCITY;
    //ldrive_sp_act = -m_frontright_drive.getSelectedSensorVelocity()/kDRIVE.MAX_VELOCITY;
    //chassis_sp = (rdrive_sp_act + ldrive_sp_act)/2;
    
    // Scale turn portion with speed
    //double turnScale = 1-(kDRIVE.TURN_SPEED_SF*(chassis_sp));
    double turnScale = 1;
    rdrive_sp_cmd = (speed + turn*turnScale);
    ldrive_sp_cmd = -(speed - turn*turnScale);
   
    // Set Drive Motor Output in Velocity Control Mode    
    m_frontleft_drive.set(ControlMode.Velocity, ldrive_sp_cmd*kDRIVE.MAX_VELOCITY);  
    m_frontright_drive.set(ControlMode.Velocity, rdrive_sp_cmd*kDRIVE.MAX_VELOCITY);

  }

  public void positionDrive(double right, double left){
    m_frontright_drive.set(ControlMode.Position, right);
    m_frontleft_drive.set(ControlMode.Position, left);
  }

  public double getPostion(boolean side){
    if (side == true){
      return m_frontleft_drive.getSelectedSensorPosition();
    }
    else {
      return m_frontright_drive.getSelectedSensorPosition();
    }
  }

  public double getMotorOutput(){
    return m_frontleft_drive.getMotorOutputPercent();
  }

  public void setMotorOutput(double output){
    m_frontleft_drive.set(ControlMode.PercentOutput, -output);
    m_frontright_drive.set(ControlMode.PercentOutput, output);
  }
  
  /*
    public void updateDashboard(){
    SmartDashboard.putNumber(kDASHBOARD.rdrive_sp_cmd, -rdrive_sp_cmd);
    SmartDashboard.putNumber(kDASHBOARD.rdrive_sp_act, rdrive_sp_act);
    SmartDashboard.putNumber(kDASHBOARD.ldrive_sp_cmd, ldrive_sp_cmd);
    SmartDashboard.putNumber(kDASHBOARD.ldrive_sp_act, ldrive_sp_act );
    SmartDashboard.putNumber(kDASHBOARD.chassis_sp, chassis_sp * kDRIVE.MAX_FTS);
  } 
  */

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
    /*
    loop_time = (loop_time++)%50;

    // Update Dashboard Values
    if (loop_time == 0 || loop_time == 25) {
      updateDashboard();
    } 
    */

  }


}
