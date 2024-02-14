// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.Auto.HighFender;
import frc.robot.commands.Auto.OneBallFender;
import frc.robot.commands.Auto.OneBallHigh;
import frc.robot.commands.Auto.OneBallLow;
import frc.robot.commands.Auto.OneBallTarmac;
import frc.robot.commands.Climber.AutoClimb;
import frc.robot.commands.Climber.ManualClimb;
import frc.robot.commands.Climber.ToggleClimbLock;
import frc.robot.commands.Drive.DriveFoward;
import frc.robot.commands.Drive.DriveTeleop;
import frc.robot.commands.Drive.DriveXY;
import frc.robot.commands.Elevator.ManualCallElevator;
import frc.robot.commands.FrontIntake.ManualFrontIntake;
import frc.robot.commands.FrontIntake.Passthrough;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Elevator;
import frc.robot.commands.Intake.ManualAdvance;
import frc.robot.commands.Intake.ManualIntake;
import frc.robot.commands.Intake.SearchBall;
import frc.robot.subsystems.ubIntake;
import frc.robot.subsystems.FrontIntake;
import frc.robot.commands.Shooter.HighTrim;
import frc.robot.commands.Shooter.SetShooterSpeed;
import frc.robot.commands.Shooter.Shoot;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SpeedyPuffJunior;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  //#region   Declare Class Variables
    // Limelight
  private final LimeLight limelight = new LimeLight();
  
  // Controllers
  private final static XboxController drivecontroller_p0 = new XboxController(0);
  private final static XboxController drivecontroller_p1 = new XboxController(1);
  
  // The robot's subsystems and commands are defined here... 
  // Subsystems
  private final SpeedyPuffJunior s_speedypuffjunior = new SpeedyPuffJunior();   // Drive
  private final Elevator s_elevator = new Elevator();
  private final ubIntake s_ubIntake = new ubIntake();
  private final FrontIntake s_frontIntake = new FrontIntake();
  private final Shooter s_shooter = new Shooter();
  private final Climber s_climber = new Climber();

  // Drive Commands
  //private final DriveTeleop c_driveTeleop = new DriveTeleop(s_speedypuffjunior);
    // Parameterised Commands:
    // DriveTeleop(speed, turn) - Default Command feed controller axis methods
    // DriveXY(X, Y)      - Rotate and drive to relative position
    // DriveFoward(dist)  - Drive forward a set distance (used in DriveXY)
    // FaceAngle(angle)   - Rotate to a set angle (used in DriveXY)

  // Compressor
  private final Compressor compressor = new Compressor(PneumaticsModuleType.CTREPCM);

  // Autonomous chooser
  private SendableChooser<Command> m_AutonChooser = new SendableChooser<>();

  // Shooter chooser
  //private SendableChooser<Double> m_ShootChooser = new SendableChooser<>();

  
  //#endregion

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    //#region Default Commands
    s_speedypuffjunior.setDefaultCommand(new DriveTeleop(   // Drive Default Command
        s_speedypuffjunior,                                   // Subsystem
        drivecontroller_p0::getLeftY,                         // Speed Axis (Method Reference)
        drivecontroller_p0::getRightX));                      // Turn Axis (Method Reference)
    /*s_ubIntake.setDefaultCommand(new SearchBall(            // Underbody Intake Default Command
        s_ubIntake,                                           // Subsystem
        s_elevator::isUp)); */                                  // Elevator State (Method Reference)
    s_climber.setDefaultCommand(new ManualClimb(            // Climber Default Command
        s_climber,                                            // Subsystem
        drivecontroller_p1::getLeftY,                         // Climber Motor Joystick Axis (Method Reference)
        drivecontroller_p1::getBackButtonPressed,             // Previous Claw State Button (Method Reference)
        drivecontroller_p1::getStartButtonPressed));          // Next Claw State Button (Method Reference)

    compressor.enableDigital(); //Starts compressor

    //#endregion

    // Configure the button bindings
    configureButtonBindings();

    /** Config Dashboard */

    // Set up Shooter Chooser for Taxi and Shoot
    /*m_ShootChooser.addOption("Low Goal Fender", kShooter.fntLG_FENDER_SP);
    m_ShootChooser.addOption("High Goal Fender", kShooter.fntHG_FENDER_SP);
    m_ShootChooser.addOption("High Goal Tarmac", kShooter.fntHG_TARMAC_SP);
    */

    // Set up Autonomous Chooser
    //m_AutonChooser.setDefaultOption("One Ball Fender", new OneBallFender(s_speedypuffjunior, s_shooter, s_elevator, s_ubIntake, s_frontIntake, 0, -8)); // subsystems, rel X, rel Y
    //m_AutonChooser.addOption("One Ball Tarmac", new OneBallTarmac(s_speedypuffjunior, s_shooter, s_elevator, s_ubIntake, s_frontIntake, 0, -2)); // subsystems, rel X, rel Y
    //m_AutonChooser.addOption("Two Ball Fender", new TwoBallFender(s_speedypuffjunior, s_shooter, s_elevator, s_ubIntake, s_frontIntake, 0, -8));

    m_AutonChooser.setDefaultOption("Taxi", new DriveFoward(s_speedypuffjunior, -8));
    m_AutonChooser.addOption("One Ball Low Goal", new OneBallLow(s_speedypuffjunior, s_shooter, s_elevator));
    m_AutonChooser.addOption("High Goal Fender", new HighFender(s_speedypuffjunior, s_shooter, s_elevator));
    m_AutonChooser.addOption("High Goal Tarmac", new OneBallHigh(s_speedypuffjunior, s_shooter, s_elevator));
    m_AutonChooser.addOption("Nothing", null);
    //m_AutonChooser.addOption("Drive to XY", new DriveXY(s_speedypuffjunior, 0, -4));
    

    Shuffleboard.getTab("Autonomous").add(m_AutonChooser);
    //Shuffleboard.getTab("Autonomous").add(m_ShootChooser);
  }

//#region Controllers

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    
    // Define Drive Controller
    Trigger leftTrigger_p0 = new Trigger(this::isLeftTrigger0);
    Trigger rightTrigger_p0 = new Trigger(this::isRightTrigger0);
    JoystickButton aButton_p0 = new JoystickButton(drivecontroller_p1, Button.kA.value);
    POVButton up_Dpad_p0 = new POVButton(drivecontroller_p0, 0);
    POVButton down_Dpad_p0 = new POVButton(drivecontroller_p0, 180);

    rightTrigger_p0.whileActiveContinuous(new Passthrough(s_frontIntake));  // Run Front roller only
    leftTrigger_p0.whileActiveContinuous(new ManualFrontIntake(s_frontIntake, s_ubIntake)); 
    aButton_p0.whenPressed(new Shoot(s_shooter, s_elevator, s_ubIntake));
    up_Dpad_p0.whenPressed(new HighTrim(s_shooter, 0.05)); // Increase shooter by 5%
    down_Dpad_p0.whenPressed(new HighTrim(s_shooter, -0.05)); // Decrease shooter by 5%


    // Define Manip Controller
    Trigger rightTrigger_p1 = new Trigger(this::isRightTrigger1); 
    Trigger leftTrigger_p1 = new Trigger(this::isLeftTrigger1);

    JoystickButton rbButton_p1 = new JoystickButton(drivecontroller_p1, Button.kRightBumper.value);
    JoystickButton lbButton_p1 = new JoystickButton(drivecontroller_p1, Button.kLeftBumper.value);
    
    JoystickButton yButton_p1 = new JoystickButton(drivecontroller_p1, Button.kY.value);
    JoystickButton bButton_p1 = new JoystickButton(drivecontroller_p1, Button.kB.value);
    JoystickButton aButton_p1 = new JoystickButton(drivecontroller_p1, Button.kA.value);
    JoystickButton xButton_p1 = new JoystickButton(drivecontroller_p1, Button.kX.value);
    
    JoystickButton rStick_p1 = new JoystickButton(drivecontroller_p1, Button.kRightStick.value);
    JoystickButton lStick_p1 = new JoystickButton(drivecontroller_p1, Button.kLeftStick.value);

    POVButton left_Dpad = new POVButton(drivecontroller_p1, 270);
    POVButton down_Dpad_p1 = new POVButton(drivecontroller_p1, 180);
    POVButton right_Dpad_p1 = new POVButton(drivecontroller_p1, 90);
    POVButton up_Dpad_p1 = new POVButton(drivecontroller_p1, 0);

    /** DO NOT USE - PASSED TO DEFAULT CLIMBER COMMAND 
     *  drivecontroller_p1 Left Y
     *  JoystickButton back_p1 = new JoystickButton(drivecontroller_p1, Button.kBack.value);
     *  JoystickButton start_p1 = new JoystickButton(drivecontroller_p1, Button.kStart.value);
    */

    rightTrigger_p1.whileActiveContinuous(new Passthrough(s_frontIntake));     // Manual underbot intake
    leftTrigger_p1.whileActiveContinuous(new ManualFrontIntake(s_frontIntake, s_ubIntake));  // Run Front Intake

    rbButton_p1.whileHeld(new ManualCallElevator(s_elevator));        // Toggle Elevator Position
    lbButton_p1.whenPressed(new ManualAdvance(s_ubIntake));             // Run Ball Advance Routine

    yButton_p1.whenPressed(new InstantCommand(s_shooter::changeSP,s_shooter));     // Change shooter setpoint      
    aButton_p1.whenPressed(new Shoot(s_shooter, s_elevator, s_ubIntake));         // Auto Shoot
    //xButton_p1.whenPressed(new AutoClimb(s_climber));                             // Auto climb
    bButton_p1.whileHeld(new ManualIntake(s_ubIntake));

    rStick_p1.whenPressed(new SetShooterSpeed(s_shooter, 0, 0));     // Stop shooter wheels

    lStick_p1.whenPressed(new ToggleClimbLock(s_climber));          // Toggle Climber Lock
    // Manual Climb is handled by the Default Command

    up_Dpad_p1.whenPressed(new HighTrim(s_shooter, 0.05));
    down_Dpad_p1.whenPressed(new HighTrim(s_shooter, -0.05));
    //rightTrigger_p1.whenPressed(new ManualCallElevator(subsystem))
    right_Dpad_p1.whenPressed(new InstantCommand(s_shooter::spinUP, s_shooter)); // Manual spin up shooter
    

  }

  public boolean isLeftTrigger0(){
    return drivecontroller_p0.getLeftTriggerAxis() > 0.3;
  }

  public boolean isLeftTrigger1(){
    return drivecontroller_p1.getLeftTriggerAxis() > 0.3;
  }

  public boolean isRightTrigger1(){
    return drivecontroller_p1.getRightTriggerAxis() > 0.3;
  }

  public boolean isRightTrigger0(){
    return drivecontroller_p0.getRightTriggerAxis() > 0.3;
  }

  public static double GetDriveAxis(int axis){
    return drivecontroller_p0.getRawAxis(axis);

  }

  

//#endregion

//#region LimeLight
  public void updateLimeLight(){
    limelight.UpdateLimeLight();
  }
//#endregion

//#region Autonomous

/**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    
    return m_AutonChooser.getSelected();
    //return c_DriveXY;
  }
//#endregion

}
