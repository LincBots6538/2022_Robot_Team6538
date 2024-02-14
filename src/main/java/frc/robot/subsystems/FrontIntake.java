// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.kFrontIntake;

public class FrontIntake extends SubsystemBase {
  /** Creates a new FrontIntake. 
   *  FrontIntake is an Intake Arm on the front of the robot, to feed balls to the under body intake when the robot can not 
   *  just drive over them (e.g. against a wall or other object). The intake is lowered into position by a piston, and the
   *  roller is powered by a BAG Motor / Talon SRX
  */
  private TalonSRX m_frontIntake = new TalonSRX(kFrontIntake.FRONT_INTAKE_ID);

  private DoubleSolenoid solenoid = new DoubleSolenoid(kFrontIntake.PCM_ID, PneumaticsModuleType.CTREPCM, kFrontIntake.FOWARD_DOUBLE_SOLENIOD_CHANNEL, kFrontIntake.REVERSE_DOUBLE_SOLENIOD_CHANNEL);

  public FrontIntake() {
    m_frontIntake.set(ControlMode.PercentOutput, 0);
    solenoid.set(kFrontIntake.INTAKE_UP);
  }

  public void setUp(){
    m_frontIntake.set(ControlMode.PercentOutput, 0);
    solenoid.set(kFrontIntake.INTAKE_UP);
  }

  public void setDown(){
    m_frontIntake.set(ControlMode.PercentOutput, kFrontIntake.ROLLER_INTAKE_CW);
    solenoid.set(kFrontIntake.INTAKE_DOWN);
  }

  public boolean isUp(){
    DoubleSolenoid.Value state = solenoid.get();
    if(state == kFrontIntake.INTAKE_UP){
      return true;
    }
    else{
      return false;
    }
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
