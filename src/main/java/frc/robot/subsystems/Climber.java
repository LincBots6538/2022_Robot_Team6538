// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.kClimber;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  private TalonSRX m_rightClimber = new TalonSRX(kClimber.RIGHT_CLIMBER_MOTOR);   // Lead Controller
  private TalonSRX m_leftClimber = new TalonSRX(kClimber.LEFT_CLIMBER_MOTOR);     // Follower
  
  // fwd - Hook face forward to catch the bar
  // bck - Hook that closes to capture the bar
  // Claw A - Side that grabs the first bar (MID), and last bar (Transverse)
  private DoubleSolenoid p_clawA_fwd = new DoubleSolenoid(kClimber.PCM_ID, PneumaticsModuleType.CTREPCM, kClimber.HOOK_A_FWD_OPEN, kClimber.HOOK_A_FWD_CLOSED);
  private DoubleSolenoid p_clawA_bck = new DoubleSolenoid(kClimber.PCM_ID, PneumaticsModuleType.CTREPCM, kClimber.HOOK_A_BCK_OPEN, kClimber.HOOK_A_BCK_CLOSED);
  // Claw B - Side that grabs the intermidate bar (High)
  private DoubleSolenoid p_clawB_fwd = new DoubleSolenoid(kClimber.PCM_ID, PneumaticsModuleType.CTREPCM, kClimber.HOOK_B_FWD_OPEN, kClimber.HOOK_B_FWD_CLOSED);
  private DoubleSolenoid p_clawB_bck = new DoubleSolenoid(kClimber.PCM_ID, PneumaticsModuleType.CTREPCM, kClimber.HOOK_B_BCK_OPEN, kClimber.HOOK_B_BCK_CLOSED);

  // Hook Switches
  private DigitalInput s_rightClawA = new DigitalInput(2);
  private DigitalInput s_leftClawA = new DigitalInput(3);
  private DigitalInput s_rightClawB = new DigitalInput(4);
  private DigitalInput s_leftClawB = new DigitalInput(5);


  // Restrict Climber unless operation is confirmed (by controller button)
  private boolean locked = true;

  private double climber_act, climber_cmd;

  // Shuffleboard Infor
  private ShuffleboardTab db_ClimberGroup = Shuffleboard.getTab("Climber");
    private NetworkTableEntry db_ClimberLock = 
      db_ClimberGroup.add("Climber Lock", true)
        .getEntry();
    private NetworkTableEntry db_ClawA = 
      db_ClimberGroup.add("Climber Claw A State", "Pass-Through")
        .getEntry();
    private NetworkTableEntry db_ClawB = 
      db_ClimberGroup.add("Climber Claw B State", "Pass-Through")
        .getEntry();
    private NetworkTableEntry db_BarA = 
      db_ClimberGroup.add("On BarA", false)
        .getEntry();
    private NetworkTableEntry db_BarB = 
      db_ClimberGroup.add("On BarB", false)
      .getEntry();

  public Climber() {
    m_rightClimber.configFactoryDefault();
    m_leftClimber.configFactoryDefault();
    m_rightClimber.setNeutralMode(NeutralMode.Brake);
    m_leftClimber.setNeutralMode(NeutralMode.Brake);
    m_leftClimber.setInverted(true);

    //Slot 0
    m_rightClimber.config_kP(0, kClimber.P);
    m_rightClimber.config_kF(0, kClimber.Ff);
    m_rightClimber.configAllowableClosedloopError(0, kClimber.PID_ERROR);

    m_leftClimber.config_kP(0, kClimber.P);
    m_leftClimber.config_kF(0, kClimber.Ff);
    m_leftClimber.configAllowableClosedloopError(0, kClimber.PID_ERROR);

    m_leftClimber.configClosedLoopPeakOutput(0, 1);
    m_rightClimber.configClosedLoopPeakOutput(0, 1);

    // Slot 1
    m_rightClimber.config_kP(1, kClimber.P);
    m_rightClimber.config_kF(1, kClimber.Ff);
    m_rightClimber.configAllowableClosedloopError(1, kClimber.PID_ERROR);

    m_leftClimber.config_kP(1, kClimber.P);
    m_leftClimber.config_kF(1, kClimber.Ff);
    m_leftClimber.configAllowableClosedloopError(1, kClimber.PID_ERROR);

    m_leftClimber.configClosedLoopPeakOutput(1, 1);
    m_rightClimber.configClosedLoopPeakOutput(1, 1);

    // reset Sensor
    m_leftClimber.setSelectedSensorPosition(0);
    m_rightClimber.setSelectedSensorPosition(0);

    // Select PID slot
    m_rightClimber.selectProfileSlot(0, 0);
    m_leftClimber.selectProfileSlot(0, 0);
  }

  public void lock(boolean state){
    locked = state;
    m_rightClimber.set(ControlMode.PercentOutput, 0);
    db_ClimberLock.setBoolean(locked);
  }

  public boolean islocked(){
    return locked;
  }

  // Check if clawA is against a bar
  public boolean isBarA(){
    if (s_leftClawA.get() == false && s_rightClawA.get() == false){
      return true;
    }
    else {
      return false;
    }
  }

  // Check if clawA is against a bar
  public boolean isBarB(){
    if (s_leftClawB.get() == false && s_rightClawB.get() == false){
      return true;
    }
    else {
      return false;
    }
  }

  public void setClawA(int state){
    if (locked == false){
      switch (state) {
        case 0: // Open - Ready for next bar
          p_clawA_fwd.set(kClimber.HOOK_CLOSED);
          p_clawA_bck.set(kClimber.HOOK_OPEN);
          db_ClawA.setString("Open");
          break;
        case 1: // Closed - Around a Bar
          p_clawA_fwd.set(kClimber.HOOK_CLOSED);
          p_clawA_bck.set(kClimber.HOOK_CLOSED);
          db_ClawA.setString("Closed");
          break;
        case 2: // Pass Through - Both Sides Open to Fall through
          p_clawA_fwd.set(kClimber.HOOK_OPEN);
          p_clawA_bck.set(kClimber.HOOK_OPEN);
          db_ClawA.setString("Pass-Through");
          break;
      }
    }
  }

  public void setClawB(int state){
    if (locked == false) {
      switch (state) {
        case 0: // Open
          p_clawB_fwd.set(DoubleSolenoid.Value.kReverse);
          p_clawB_bck.set(DoubleSolenoid.Value.kForward);
          db_ClawA.setString("Open");
          break;
        case 1: // Closed
          p_clawB_fwd.set(DoubleSolenoid.Value.kReverse);
          p_clawB_bck.set(DoubleSolenoid.Value.kReverse);
          db_ClawB.setString("Closed");
          break;
        case 2: // Pass Through
          p_clawB_fwd.set(DoubleSolenoid.Value.kForward);
          p_clawB_bck.set(DoubleSolenoid.Value.kForward);
          db_ClawB.setString("Pass-Through");
          break;
      }
    }
  }

  public void setMotor(double velocity){
    if (locked == false) {
    //climber_act = m_rightClimber.getSelectedSensorVelocity();

    //Velocity Control
    //m_rightClimber.set(ControlMode.Velocity, velocity);
    //m_leftClimber.set(ControlMode.Velocity, velocity);

    //Raw Output
    m_rightClimber.set(ControlMode.PercentOutput, velocity);
    m_leftClimber.set(ControlMode.PercentOutput, velocity);
    
    //positional Control
    /*
    climber_act = m_leftClimber.getSelectedSensorPosition();
    climber_cmd = climber_act + velocity*100;

    m_leftClimber.set(ControlMode.Position, climber_cmd);
    m_rightClimber.set(ControlMode.Position, climber_cmd);
    */
    }
  }

  public void resetPosition(){
    m_rightClimber.setSelectedSensorPosition(0, 0, 100);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (!locked){
      db_BarA.setBoolean(isBarA());
      db_BarB.setBoolean(isBarB());

      SmartDashboard.putNumber("Climber Left Output", m_leftClimber.getMotorOutputPercent());
      SmartDashboard.putNumber("Climber Right Output", m_rightClimber.getMotorOutputPercent());
    }
  }
}
