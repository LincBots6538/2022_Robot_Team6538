// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;


import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.SpeedyPuffJunior;
import frc.robot.subsystems.Shooter;
import frc.robot.Constants.kShooter;
import frc.robot.commands.Drive.DriveFoward;
import frc.robot.commands.Drive.FaceAngle;
import frc.robot.commands.Elevator.ManualCallElevator;
import frc.robot.commands.Shooter.SetShooterSpeed;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.FrontIntake;
import frc.robot.subsystems.ubIntake;


public class OneBallFender extends SequentialCommandGroup {
  /** Creates a new TaxiandShoot. */

  //private SendableChooser<Double> m_ShootChooser;
  
  public OneBallFender(SpeedyPuffJunior s_speedypuffjunior, Shooter s_shooter, Elevator s_elevator, ubIntake s_ubIntake, FrontIntake s_frontIntake, double relX, double relY) {
    // Add your commands in the addCommands() call, e.g.
    
    // Calculates degree and distance relative to an X and Y coordinate
    // Converts radians to degrees

    //m_ShootChooser = shootChooser;
    //double rpm = m_ShootChooser.getSelected();

    double distance = Math.sqrt(Math.pow((relX), 2) + Math.pow((relY), 2));

    double degree = Math.toDegrees(Math.acos(relY/distance));
    
    if(relX < 0){
        degree = -degree;
    }

     SmartDashboard.putNumber("distance", distance);
     SmartDashboard.putNumber("degree", degree);
     
    addCommands(
      // Sets Shooter speed to High goal fender
      new SetShooterSpeed(s_shooter, kShooter.fntHG_FENDER_SP, kShooter.bckHG_FENDER_SP),

      // Pushes Ball into shooter
      new ManualCallElevator(s_elevator),

      // Calls elevator back down
      new ManualCallElevator(s_elevator),

      new SetShooterSpeed(s_shooter, 0, 0),

      new FaceAngle(s_speedypuffjunior, degree),

      new DriveFoward(s_speedypuffjunior, distance)
    );
  }
}
