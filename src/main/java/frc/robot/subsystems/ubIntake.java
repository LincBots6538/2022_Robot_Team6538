// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.kubIntake;

public class ubIntake extends SubsystemBase {
  /** Creates a new ubIntake. 
   *  ubIntake is the Under Body Intake. The design is a pair of rollers each mounted on a pivoting arm actuated by a piston 
   *  and each roller powered by a BAG motot / Talon SRX combination. REV Color sensors are used in front of 
   *  
   *  This system is cabale of:
   *  1) Intaking the correct color ball from under the robot
   *  2) Bypassing the wrong color balls
   *  3) Raising to a stored position
   *  4) Advancing the stored ball to the loader system
   * 
   */
    private TalonSRX m_frontIntake = new TalonSRX(kubIntake.UB_FRONT_INTAKE_ID);
    private TalonSRX m_rearIntake = new TalonSRX(kubIntake.UB_BACK_INTAKE_ID);

    private Solenoid solenoid_front = new Solenoid(kubIntake.PCM_ID, PneumaticsModuleType.CTREPCM, kubIntake.FRONT_SINGLE_SOLENOID_CHANNEL);
    private Solenoid solenoid_rear = new Solenoid(kubIntake.PCM_ID, PneumaticsModuleType.CTREPCM, kubIntake.REAR_SINGLE_SOLENOID_CHANNEL);

    private I2C.Port i2cPort = I2C.Port.kMXP;
    private ColorSensorV3 s_color = new ColorSensorV3(i2cPort);
    private Color ballColor;

    private DigitalInput s_storedBall = new DigitalInput(0);
    private DigitalInput s_loadedBall = new DigitalInput(1);

    private int baseRed, baseBlue;
    private DriverStation.Alliance alliance;

    private boolean busyflag = false;
    private Timer busyTimer = new Timer();
    private Timer dbTimer = new Timer();

    private boolean change = true;
    private int state = 0;
    private double delay = kubIntake.STATE_DELAY;

    private boolean sensorError = false;
  
    private ShuffleboardTab tab = Shuffleboard.getTab("Comp");
    private NetworkTableEntry db_stored = 
      tab.add("Ball Stored", false)
         .getEntry();
    private NetworkTableEntry db_ready = 
    tab.add("Ball Ready", false)
      .getEntry();

  public ubIntake() {
    baseRed = s_color.getRed();
    baseBlue = s_color.getBlue();
    alliance = DriverStation.getAlliance(); 
    dbTimer.reset();
    dbTimer.start();
  }

//#region States

  public void setIntaking(){
    state = 1;  // Start Motors, then move to Intaking
    change = true;
  }

  public void setStored(){
    if (state == 1){
      state = 4;
    }
    else {
      state = 0;
    }
    change = true;
  }

  public void setAdvance(){
    state = 2;
    change = true;
  }

  public void setSearching(){
    state = 5;
    change = true;
  }

  public void setPassUnder(){
    state = 7;
    change = true;
  }

//#endregion

//#region Sensor Functions
public int getCurrentState(){
  return state;
}

public double getRedRaw(){
  return s_color.getRed();
}

public boolean isBusy(){
  return (busyflag || change);
}

public boolean isBallUnder(){
  return (kubIntake.PROX_THRESHOLD > s_color.getProximity());
}

public boolean isBallStored(){
  return ! s_storedBall.get();
}

public boolean isBallLoaded(){
  return ! s_loadedBall.get();
}

public boolean isOffensive(){
  if (alliance == DriverStation.Alliance.Red){
    return (baseRed + kubIntake.DELTA_COLOR_THRESHOLD < ballColor.red);
  }
  else {
    return (baseBlue + kubIntake.DELTA_COLOR_THRESHOLD < ballColor.blue);
  }
}

public boolean isDefensive(){
  if (alliance == DriverStation.Alliance.Red){
    return (baseBlue + kubIntake.DELTA_COLOR_THRESHOLD < ballColor.blue);
  }
  else {
    return (baseRed + kubIntake.DELTA_COLOR_THRESHOLD < ballColor.red);
  }
}



// set the Sensor Error Flag
public void setSensorError(boolean error){
  sensorError = error;
}

public boolean getSensorError(){
  return sensorError;
}

public void updateDashboard(){
    db_ready.setBoolean(isBallLoaded());
    db_stored.setBoolean(isBallStored());
}



//#endregion
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (dbTimer.get() >.1){
      updateDashboard();
      dbTimer.reset();
    }
    
    ballColor = s_color.getColor();

    if (busyflag && busyTimer.get() > delay){
      busyflag = false;
      busyTimer.stop();
      busyTimer.reset();
    }
    else if (busyflag == false && change){
      switch (state) {
        case 0:   // Stored
          solenoid_front.set(kubIntake.UBARMS_UP);
          m_frontIntake.set(ControlMode.PercentOutput, 0);
          solenoid_rear.set(kubIntake.UBARMS_UP);
          m_rearIntake.set(ControlMode.PercentOutput, 0);
          busyflag = true;
          busyTimer.start();
          change = false;
          delay = kubIntake.STATE_DELAY;
          break;
        case 1:   // Intaking
          solenoid_front.set(kubIntake.UBARMS_DOWN);
          m_frontIntake.set(ControlMode.PercentOutput, kubIntake.ROLLER_INTAKE_CW);
          solenoid_rear.set(kubIntake.UBARMS_DOWN);
          m_rearIntake.set(ControlMode.PercentOutput, kubIntake.ROLLER_INTAKE_CW);
          busyflag = true;
          busyTimer.start();
          change = false;
          delay = kubIntake.STATE_DELAY;
          break;
        case 2:   // Advance
          solenoid_front.set(kubIntake.UBARMS_UP);
          m_frontIntake.set(ControlMode.PercentOutput,- kubIntake.ROLLER_ADVANCE_CW);
          solenoid_rear.set(kubIntake.UBARMS_UP);
          m_rearIntake.set(ControlMode.PercentOutput, kubIntake.ROLLER_ADVANCE_CW);
          busyflag = true;
          busyTimer.start();
          state = 3;
          delay = kubIntake.ADVANCE_DELAY;
          break;
        case 3:   // Kick
          solenoid_front.set(kubIntake.UBARMS_UP);
          m_frontIntake.set(ControlMode.PercentOutput, 0);
          solenoid_rear.set(kubIntake.UBARMS_DOWN);
          m_rearIntake.set(ControlMode.PercentOutput, 0);
          busyflag = true;
          busyTimer.start();
          state = 0;    // Go to Stored
          delay = kubIntake.KICK_DELAY;
          break;
        case 4:   // Pick Up
          solenoid_front.set(kubIntake.UBARMS_UP);
          m_frontIntake.set(ControlMode.PercentOutput, kubIntake.ROLLER_INTAKE_CW);
          solenoid_rear.set(kubIntake.UBARMS_UP);
          m_rearIntake.set(ControlMode.PercentOutput, kubIntake.ROLLER_INTAKE_CW);
          busyflag = true;
          busyTimer.start();
          state = 0;    // Go to Stored
          delay = kubIntake.STATE_DELAY;
          break;
        case 5: // Searching
          solenoid_front.set(kubIntake.UBARMS_UP);
          m_frontIntake.set(ControlMode.PercentOutput, 0);
          solenoid_rear.set(kubIntake.UBARMS_DOWN);
          m_rearIntake.set(ControlMode.PercentOutput, 0);
          busyflag = true;
          busyTimer.start();
          change = false;
          delay = kubIntake.STATE_DELAY;
          break;
        case 6: // Pre Intake
          // Leave Arms in Place
          m_frontIntake.set(ControlMode.PercentOutput, kubIntake.ROLLER_INTAKE_CW);
          m_rearIntake.set(ControlMode.PercentOutput, kubIntake.ROLLER_INTAKE_CW);
          busyflag = true;
          busyTimer.start();
          state = 1;    // Go to Intake
          delay = kubIntake.MOTOR_DELAY;
          break;
        case 7: // Pass Ball Under
          solenoid_front.set(kubIntake.UBARMS_UP);
          m_frontIntake.set(ControlMode.PercentOutput, 0);
          solenoid_rear.set(kubIntake.UBARMS_DOWN);
          m_rearIntake.set(ControlMode.PercentOutput, -kubIntake.ROLLER_INTAKE_CW);
          busyflag = true;
          busyTimer.start();
          state = 5; // Go to Searching
          delay = kubIntake.ADVANCE_DELAY;
          break;
      }
    }
  }
}
