// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{
  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag
  public static final double MAX_SPEED  = Units.feetToMeters(14.5);// Maximum speed of the robot in meters per second, used to limit acceleration.

  public static final class ElevatorConstants
  {
    public static final int leftElevatorID = 11;  // Can Id of Left elevator motor
    public static final int rightElevatorID = 12; // Can Id of Right elevator motor

    public static final int MaxCurrentLimit = 40; // Max current limit of elevator motors 
    public static final double maxOutput = 1.0;   // Max power output of elevator motors
    public static final double maxVelocity = 8000.0; // Max velocity of elevator
    public static final double maxAcceleration = 10000.0; // Max acceleration of elevator
    public static final double countsPerInch = 0.9861;   // Encoder ticks per inch of elevator travel
    public static final double posTolerance = 0.1;    // Tolerance for PID control of elevator

    // PID values
    public static final double kP = 0.05;
    public static final double kI = 0.0;
    public static final double kD = 0.01;

    // Feedforward values
    public static final double kAF = 0.02;
    
    // Stored elevator positions
    public static final double minPos = 5;
    public static final double downPos = 2.0;
    public static final double L1 = 7.0;
    public static final double L2 = 20.0;
    public static final double L3 = 25.0;
    public static final double L4 = 35.0;
    public static final double maxPos = 40.0;
  }

  public static final class DrivebaseConstants
  {
    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static class OperatorConstants
  {
    // Joystick Deadband
    public static final double DEADBAND        = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT    = 6;
  }
}
