// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DoubleSolenoid;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    // Constant Variable Names should be in ALL CAPS, and create a new class for each subsystem

    public static final class kDRIVE {
        // Device CAN IDs
        public static final int FRONT_LEFT_CANID = 3;         // Falcon 500 - TalonFX
        public static final int REAR_LEFT_CANID = 2;          // Falcon 500 - TalonFX
        public static final int FRONT_RIGHT_CANID = 1;        // Falcon 500 - TalonFX
        public static final int REAR_RIGHT_CANID =0;          // Falcon 500 - TalonFX

        // Hardware Parameters
        public static final double MAX_VELOCITY = 20000;      // Motor velocity in encoder ticks / 100ms
        public static final double MAX_FTS = 14.27;           // Drive Max Speed in Ft/s
        public static final double RAMP_RATE = 1;             // Time in seconds to accel from neutral to full speed 
        public static final double TURN_SPEED_SF = 0.75;      // Scale Factor to compansate Turning input for speed
        public static final double WHEEL_DIAMETER = 6;        // Wheel Diameter in inches
        public static final double GEAR_RATIO = 10.75;        // Drive Gear Ratio
        public static final double WIDTH = 26;                // Track Width

        // Input Parameters
        public static final double JOYSTICK_DEADBAND = 0.1;   // Deadband for Joystick Axes
        public static final int SPEED_AXIS = 1;               // Joystick Axis for drive speed input
        public static final int TURN_AXIS = 4;                // Joystick Axis for drive turning input

        // PID Values in Phoenix Tuner for slot 0
        public static final double KP_value_s0 = 0.005;           // KP value in phoenix tuner
        public static final double KF_value_s0 = 0.05;           // KF value in phoenix tuner
        public static final double PID_Error_s0 = 100;           // Sets the PID Error margin 
        
        //PID Values in Phoenix Tuner for slot 1
        public static final double KP_value_s1 = 0.05;
        public static final int KF_value_s1 = 0;           
        public static final double PID_Error_s1 = 100;

        //PID Values in Phoenix Tuner for slot 2
        public static final double KP_value_s2 = 0.02;
        public static final int KF_value_s2 = 0;           
        public static final double PID_Error_s2 = 25;

        
    }

    public static final class kDASHBOARD {
        // These constants are named the same as their corresponding value varibles
        //Drive
        public static final String rdrive_sp_act = "Drive Right Side Actual";
        public static final String rdrive_sp_cmd = "Drive Right Side Command";
        public static final String ldrive_sp_act = "Drive Left Side Actual";
        public static final String ldrive_sp_cmd = "Drive Left Side Command";
        public static final String chassis_sp = "Chassis Speed";

        //Shooter
        public static final String shooter_act = "Actual Shooter speed";
        public static final String shooter_cmd = "Command Shooter speed";
    }

    public static final class kShooter{
        //Device CAN IDs
        public static final int BACK_SHOOTER_ID = 12;
        public static final int FRONT_SHOOTER_ID = 13;

        public static final double WHEEL_DIAMETER = 4.0;
        public static final double GEAR_RATIO = 1.5;

        //Input Parametrs
        public static final int POVup = 0;
        public static final int POVdown = 180;

        // PID Coefficients
        public static final double P = 0.005;
        public static final double I = 0;
        public static final double D = 0;
        public static final double Iz = 0;
        public static final double FF = 0.05;
        public static final double MaxOutput = 1;
        public static final double MinOutput = -1;
        public static final double MaxSPEED = 20000; // in encoder cnts/100ms

        public static final double RAMP_RATE = 0.5;

        // Set Points
        public static final String LG_FENDER_Name = "Low Goal Fender Shot";
        public static final double fntLG_FENDER_SP =(950 * 1.5 * 2048/600) * 1.1;
        public static final double bckLG_FENDER_SP =(950 * 1.5 * 2048/600) * 1.1;
        public static final String HG_FENDER_Name = "High Goal Fender Shot";
        public static final double fntHG_FENDER_SP =(1480 * 1.5 * 2048/600) * 1.1;
        public static final double bckHG_FENDER_SP =(1650 * 1.5 * 2048/600) * 1.1;
        public static final String HG_TARMAC_Name = "High Goal Tarmac Shot";
        public static final double fntHG_TARMAC_SP =(1925 * 1.5 * 2048/600) * 1.1;
        public static final double bckHG_TARMAC_SP =(1925 * 1.5 * 2048/600) * 1.1;
        public static final double fntHG_AUTO_SP =(1965 * 1.5 * 2048/600) * 1.1;
        public static final double bckHG_AUTO_SP =(1765 * 1.5 * 2048/600) * 1.1;

        public static final double SENSOR_RPM_RATIO = (1.5 * 2048)/600 ;
    }

    public static final class kubIntake{
        public static final int UB_FRONT_INTAKE_ID = 5;
        public static final int UB_BACK_INTAKE_ID = 4;
        
        public static final int PCM_ID = 0;
        public static final int FRONT_SINGLE_SOLENOID_CHANNEL = 1;
        public static final int REAR_SINGLE_SOLENOID_CHANNEL = 0;

        public static final int DELTA_COLOR_THRESHOLD = 100;
        public static final int PROX_THRESHOLD = 1000;

        public static final boolean UBARMS_UP = true;
        public static final boolean UBARMS_DOWN = false;

        public static final double ROLLER_INTAKE_CW = 1;
        public static final double ROLLER_ADVANCE_CW = 0.9;

        public static final double ADVANCE_DELAY = 0.5;   // How long to run the motors when advancing a ball
        public static final double STATE_DELAY = 0.2;     // Delay to move Intake Arms
        public static final double KICK_DELAY = 0.5;      // Delay between Kick & returning to Stored
        public static final double MOTOR_DELAY = 0.1;     // Delay to star Motors before Moving
    
        
    }

    public static final class kFrontIntake{
        public static final int FRONT_INTAKE_ID= 7;

        public static final int PCM_ID = 0;
      
        public static final int FOWARD_DOUBLE_SOLENIOD_CHANNEL = 5;
        public static final int REVERSE_DOUBLE_SOLENIOD_CHANNEL = 4;

        public static final DoubleSolenoid.Value INTAKE_UP = DoubleSolenoid.Value.kForward;
        public static final DoubleSolenoid.Value INTAKE_DOWN = DoubleSolenoid.Value.kReverse;

        public static final double ROLLER_INTAKE_CW = 0.9;
    }

    public static final class kElevator{
        public static final int PCM_ID = 0;
        public static final int FORWARD_DOUBLE_SOLENIOD_CHANNEL = 3;
        public static final int REVERSE_DOUBLE_SOLENIOD_CHANNEL = 2;
        
        public static final DoubleSolenoid.Value ELEVATOR_UP = DoubleSolenoid.Value.kReverse;
        public static final DoubleSolenoid.Value ELEVATOR_DOWN = DoubleSolenoid.Value.kForward;

        public static final double DELAY = 0.3;
    }

    public static final class kClimber{
        public static final int LEFT_CLIMBER_MOTOR = 8;
        public static final int RIGHT_CLIMBER_MOTOR = 9;

        public static final int PCM_ID = 1;
        public static final int HOOK_A_FWD_OPEN = 7;
        public static final int HOOK_A_FWD_CLOSED = 0;

        public static final int HOOK_A_BCK_OPEN = 6;
        public static final int HOOK_A_BCK_CLOSED = 1;

        public static final int HOOK_B_FWD_OPEN = 5;
        public static final int HOOK_B_FWD_CLOSED = 2;

        public static final int HOOK_B_BCK_OPEN = 4;
        public static final int HOOK_B_BCK_CLOSED = 3;

        public static final DoubleSolenoid.Value HOOK_OPEN = DoubleSolenoid.Value.kForward;
        public static final DoubleSolenoid.Value HOOK_CLOSED = DoubleSolenoid.Value.kReverse;

        public static final int CLAW_OPEN = 0;
        public static final int CLAW_CLOSED = 1;
        public static final int CLAW_PASS = 2;

        public static final double CLIMBER_SPEED = 1.0;

        public static final double JOYSTICK_DEADBAND = 0.1;   // Deadband for Joystick Axes
        public static final double GEAR_RATIO = 5;
        public static final double ENCODER_TICKS = 4096;

        public static final double P = 0.1;
        public static final double Ff = 0.2;
        public static final double PID_ERROR = 100;
    }
}

