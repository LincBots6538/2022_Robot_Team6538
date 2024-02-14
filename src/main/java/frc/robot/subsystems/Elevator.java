// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.kElevator;

public class Elevator extends SubsystemBase {
  /** Creates a new Loader. 
   *  The Loader is a cradle to hold the ball before feeding it in to the {@link Shooter}. The cradle is 
   *  raised and lowered by a piston.
  */

  private DoubleSolenoid solenoid = new DoubleSolenoid(kElevator.PCM_ID, PneumaticsModuleType.CTREPCM, kElevator.FORWARD_DOUBLE_SOLENIOD_CHANNEL, kElevator.REVERSE_DOUBLE_SOLENIOD_CHANNEL);

  private ShuffleboardTab CompTab = Shuffleboard.getTab("Comp");
  private NetworkTableEntry db_elevator;

  public Elevator() {
    solenoid.set(kElevator.ELEVATOR_DOWN);
    db_elevator = CompTab.add("Elevator Position", "Down")
       .getEntry();

  }
  

  public void load(){
    solenoid.set(kElevator.ELEVATOR_UP);
    db_elevator.setString("Up");
  }

  public void ready(){
    solenoid.set(kElevator.ELEVATOR_DOWN);
    db_elevator.setString("Down");
  }

  public boolean isUp(){
    DoubleSolenoid.Value ele_state = solenoid.get();

    if (ele_state == kElevator.ELEVATOR_UP){
      return true;
    }
    else {
      return false;
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if(solenoid.get() == kElevator.ELEVATOR_UP){
      db_elevator.setString("Up");
    }
    else{
      db_elevator.setString("Down");
    }
  }
}
