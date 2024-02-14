// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.SpeedyPuffJunior;

/* Drive XY Commands

Robot is about 33.5 Inches (Robot frame plus the bumper)
Set A
  (Start the robot 5 inches from the tape - Back of the bumper to the side of the tape OUTSIDE the Tarmac) 
  1. Robot turns on the UBIntake, Intake lowers 
  2. Robot drives backward to pick up a ball behind it 
  3. Picks up the ball and the UBintake activates its stored position.
  4. Robot Drives foward and lines up about 4 inches from the Fender
  5. Shooter wheels spin up to speed
  6. Elevator rises to fire the 1st ball and then it retracks back to its original position.
  7. UBIntake advances the 2nd ball into the Elevator.
  8. Elevator rises again and fires the ball into the upper hub.
  9. Shooter turns off
  
*/
public class DriveXY extends SequentialCommandGroup {
  /** Creates a new DriveXY. */
  public DriveXY(SpeedyPuffJunior m_speedypuffjunior, double relX, double relY) {
    // Add your commands in the addCommands() call, e.g.
    
    // Calculates degree and distance relative to an X and Y coordinate
    // Converts radians to degrees

    double distance = Math.sqrt(Math.pow((relX), 2) + Math.pow((relY), 2));

    double degree = Math.toDegrees(Math.acos(relY/distance));
    
    if(relX < 0){
        degree = -degree;
    }

     SmartDashboard.putNumber("distance", distance);
     SmartDashboard.putNumber("degree", degree);
     
    addCommands(
      // Turn robot a set degree
      // Positive angles are in Quadrant 1 & 4
      // Negative angles are in Quadrant 2 & 3
      new FaceAngle(m_speedypuffjunior, degree),

      // Drive robot a set distance
      new DriveFoward(m_speedypuffjunior, distance)


    );
  }
}
