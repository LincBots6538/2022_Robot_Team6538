// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.kDASHBOARD;
import frc.robot.Constants.kShooter;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. 
   *  Shooter is a Pitching Wheel design that uses a pair of counter rotating shafts each driven by a REV NEO / Spark MAX controller
   *  Distance / Height is controlled via the rotational speed of the shafts. The shafts should be allowed to reach the target speed 
   *  before feeding each ball.  Spin can be added to the ball by applying different speed to each shaft.
   * 
   *  Shaft speed provided by the NEO's internal sensor
   *  Front and Rear Shafts controlled by indiviual setpoint values
  */
  private TalonFX m_shooterfront = new TalonFX(kShooter.FRONT_SHOOTER_ID);
  private TalonFX m_shooterback = new TalonFX(kShooter.BACK_SHOOTER_ID);
  private SparkMaxPIDController m_frontPID, m_rearPID;      // PID Objects for each axle
  private double s_frontEncoder, s_rearEncoder;    // Encoder Objects for each motor
  private double lowTrim =0, highTrim = 0, spin =0; 
  private double shooter_act, shooter_cmd, tgtFront, tgtRear;
  private double fntSP = 0, bckSP = 0, fnttgt = kShooter.fntLG_FENDER_SP, bcktgt = kShooter.bckLG_FENDER_SP;
  private int lowFlag = 0, highFlag = 0;
  private int loop_time, tgtState = 0;

  // Dashboard Objects
  private ShuffleboardTab CompTab = Shuffleboard.getTab("Comp");
  private ShuffleboardTab ShootTab = Shuffleboard.getTab("Shooter");
  private NetworkTableEntry db_shooterSP = 
    CompTab.add("Shooter Set Point", kShooter.LG_FENDER_Name)
       .getEntry();
  private NetworkTableEntry db_shooterReady = 
    CompTab.add("Shooter At Speed", false)
       .getEntry();
  private NetworkTableEntry db_shooterAct = 
    ShootTab.add(kDASHBOARD.shooter_act, 0)
        .getEntry();
  private NetworkTableEntry db_shooterTgt = 
    ShootTab.add(kDASHBOARD.shooter_cmd, 0)
       .getEntry();
  private NetworkTableEntry db_highTrim = 
    ShootTab.add("High Goal Trim", 0)
      .getEntry();
  private NetworkTableEntry db_highTrimComp = 
    CompTab.add("High Goal Trim", 0)
      .getEntry();
  private NetworkTableEntry db_lowTrim = 
    ShootTab.add("Low Goal Trim", 0)
      .getEntry();
  private NetworkTableEntry db_Spin = 
    ShootTab.add("Spin Trim", 0)
      .getEntry();

  public Shooter() {
    m_shooterfront.configFactoryDefault();
    m_shooterback.configFactoryDefault();

    m_shooterfront.setSelectedSensorPosition(0);
    m_shooterback.setSelectedSensorPosition(0);

    m_shooterfront.setNeutralMode(NeutralMode.Coast);
    m_shooterback.setNeutralMode(NeutralMode.Coast);

    m_shooterfront.configClosedloopRamp(kShooter.RAMP_RATE);
    m_shooterback.configClosedloopRamp(kShooter.RAMP_RATE);

    m_shooterfront.config_kP(0, kShooter.P);
    m_shooterfront.config_kF(0, kShooter.FF);
    m_shooterfront.configAllowableClosedloopError(0, 1000);

    m_shooterback.config_kP(0, kShooter.P);
    m_shooterback.config_kF(0, kShooter.FF);
    m_shooterback.configAllowableClosedloopError(0, 1000);

    m_shooterfront.configClosedLoopPeakOutput(0, 1);
    m_shooterback.configClosedLoopPeakOutput(0, 1);

    m_shooterback.setInverted(true);

  }

  // Set Base Shooter Speed
  public void setSpeed(double fnt_setpoint, double bck_setpoint){
      fntSP = fnt_setpoint;
      bckSP = bck_setpoint;
  }
  
  // Select Preset Setpoints
  public void changeSP(){
    tgtState = (tgtState +1)%3;

    switch (tgtState){
      case 0: // Low Goal Fender Shot
        fnttgt = kShooter.fntLG_FENDER_SP;
        bcktgt = kShooter.bckLG_FENDER_SP;
        setTrimFlag(false);
        db_shooterSP.setString(kShooter.LG_FENDER_Name);
        break;
      case 1: // High Goal Fender Shot
        fnttgt = kShooter.fntHG_FENDER_SP;
        bcktgt = kShooter.bckHG_FENDER_SP;
        setTrimFlag(true);
        db_shooterSP.setString(kShooter.HG_FENDER_Name);
        break;
      case 2: // High Goal Tarmac Shot
        fnttgt = kShooter.fntHG_TARMAC_SP;
        bcktgt = kShooter.bckHG_TARMAC_SP;
        setTrimFlag(true);
        db_shooterSP.setString(kShooter.HG_TARMAC_Name);
        break;
    }

    if (fntSP > 0) spinUP();
  }

  public void spinUP(){
    fntSP = fnttgt;
    bckSP = bcktgt;
  }
  
  // Set Spin Offset
  public void setSpin(double pctspin){
    spin = pctspin/2;
    db_Spin.setDouble(spin*2);
  }

  // Get Spin Offset
  public double getSpin(){
    return spin *2;
  }

  // Set Low Goal Trim
  public void setLowTrim(double pctTrim){
    lowTrim = pctTrim;
    db_lowTrim.setDouble(lowTrim);
  }
  
  // Get Low Goal Trim
  public double getLowTrim(){
    return lowTrim;
  }

  // Set High Goal Trim
  public void setHighTrim(double pctTrim){
    highTrim = pctTrim;
    db_highTrim.setDouble(highTrim);
    db_highTrimComp.setDouble(highTrim);
  }

  // Get High Goal Trim
  public double getHighTrim(){
    return highTrim;
  }
  
  // Set Trim To Use
  public void setTrimFlag(boolean high){
    if (high == true){
      highFlag = 1;
      lowFlag = 0;
    }
    else{
      highFlag = 0;
      lowFlag = 1;
    }
  }

  // Check if at Speed
  public boolean atSpeed(){
    if (shooter_act > shooter_cmd * 0.9  && shooter_act < shooter_cmd * 1.1)  return true;
    return false;
  }

//#region Remove? - Breaks ManualAdjustShootBall


//#endregion

  public void updateDashboard(){
    shooter_act = (m_shooterback.getSelectedSensorVelocity()+ m_shooterfront.getSelectedSensorVelocity())/2;
    shooter_cmd = (tgtFront + tgtRear)/2;
    db_shooterAct.setDouble(shooter_act);
    db_shooterTgt.setDouble(shooter_cmd);
    db_shooterReady.setBoolean(atSpeed());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
    // Set PID setpoint
    tgtFront = (fntSP * (1 + (highTrim * highFlag + lowTrim * lowFlag) + spin));
    tgtRear = (bckSP * (1 + (highTrim * highFlag + lowTrim * lowFlag) - spin));
    m_shooterfront.set(ControlMode.Velocity, tgtFront);
    m_shooterback.set(ControlMode.Velocity, tgtRear);
    

    loop_time = (loop_time++)%50;
    // Update Dashboard Values
    if (loop_time == 0 || loop_time == 25) {
      updateDashboard();
    }
  } 
}
