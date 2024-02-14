// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.kShooter;
import frc.robot.commands.Drive.DriveFoward;
import frc.robot.commands.Elevator.ManualCallElevator;
import frc.robot.commands.Shooter.SetShooterSpeed;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SpeedyPuffJunior;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class OneBallHigh extends SequentialCommandGroup {
  /** Creates a new OneBallLow. */
  private SpeedyPuffJunior s_speedypuffjunior;
  private Shooter s_shooter;
  private Elevator s_elevator;


  public OneBallHigh(SpeedyPuffJunior drive, Shooter shooter, Elevator elevator) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    s_speedypuffjunior = drive;
    s_shooter = shooter;
    s_elevator = elevator;

    addCommands(
      new SetShooterSpeed(s_shooter, kShooter.fntHG_AUTO_SP, kShooter.bckHG_AUTO_SP),
      //new InstantCommand(s_shooter::spinUP),
      new WaitCommand(3),
      new InstantCommand(s_elevator::load),
      new WaitCommand(1),
      new InstantCommand(s_elevator::ready),
      new SetShooterSpeed(s_shooter, 0, 0),
      new WaitCommand(5),
      new DriveFoward(s_speedypuffjunior, -7)

    );
  }
}
