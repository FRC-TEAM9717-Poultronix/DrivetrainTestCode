// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Meter;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;
import edu.wpi.first.math.numbers.N3;

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
  public static final double maxThrottle = 1.0;
  public static final double minThrottle = 0.4;

  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag
  public static final double MAX_SPEED  = Units.feetToMeters(8);// Maximum speed of the robot in meters per second, used to limit acceleration.

  public static final AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);

  public static final Pose2d startPosition = new Pose2d(new Translation2d(Meter.of(10), Meter.of(6.16)), Rotation2d.fromDegrees(0));

  public static final String Camera1Name = "center";
  public static final Rotation3d Camera1Rotation = new Rotation3d(0, Units.degreesToRadians(15), 0);
  public static final Translation3d Camera1Translation = new Translation3d(Units.inchesToMeters(13.5),
                                                                           Units.inchesToMeters(0.0),
                                                                           Units.inchesToMeters(8.0));
  public static final Vector<N3>  Camera1StdTrans = VecBuilder.fill(2, 2, 4);
  public static final Vector<N3>  Camera1StdRot = VecBuilder.fill(0.5, 0.5, 1.0);

public static final class Hanger_Constants
{
  public static final int HangerID = 30;  // Can Id of Hanger motor
  public static final double Hanger_Motor = 0.3;
}

  public static final class ElevatorConstants
  {
    public static final int leftElevatorID = 11;  // Can Id of Left elevator motor
    public static final int rightElevatorID = 12; // Can Id of Right elevator motor

    public static final int MaxCurrentLimit = 40; // Max current limit of elevator motors 
    public static final double maxOutput = 1.0;   // Max power output of elevator motors
    public static final double maxVelocity = 18000.0; // Max velocity of elevator
    public static final double maxAcceleration = 16000; // Max acceleration of elevator
    public static final double countsPerInch = 0.9861;   // Encoder ticks per inch of elevator travel
    public static final double posTolerance = 2;    // Tolerance for PID control of elevator

    // PID values
    public static final double kP = 0.1;
    public static final double kI = 0.0;
    public static final double kD = 0.02;
    public static final double kIz = 20;

    // Feedforward values
    public static final double kAF = 0.05;
    
    // Stored elevator positions
    public static final double positionMin = 31.5;  //This is more of an intial value than a position.  Travel to all other positions is the listed value minus this one.
    public static final double positionMax = 84.5;

    public static final double positionDown = 32;
    public static final double positionFloor = 36;
    public static final double positionProcessor = 34.0;
    public static final double positionA2 = 59.5;
    public static final double positionA3 = 78.0;
    public static final double positionNet = 81;
    public static final double positionLollipop = 35;
    
    public static final double positionL1 = positionMin;
    public static final double positionL2 = 39.5;
    public static final double positionL3 = 57.5;
    public static final double positionL4 = 83.5;

  }
// stored coral arm positions
  public static final class CoralConstants
  {
    public static final int launchID = 21;  
    public static final int armID =22; 

    public static final double powerLaunch = 1.0;
    public static final double powerIntake = 0.2;

    public static final int MaxCurrentLimitArm = 20; // Max current limit of elevator motors 
    public static final int MaxCurrentLimitLaunch = 20; // Max current limit of elevator motors 

    public static final double maxOutput = 1.0;   // Max power output of elevator motors
    public static final double maxVelocity = 6000.0; // Max velocity of elevator
    public static final double maxAcceleration = 30000.0; // Max acceleration of elevator
    public static final double countsPerDegreeLaunch = 360;   // Encoder ticks per inch of elevator travel
    public static final double countsPerDegreeArm = 360;   // Encoder ticks per inch of elevator travel
    public static final double posTolerance = 5.0;    // Tolerance for PID control of elevator

     // PID values
    //  public static final double kP_launch = 0.05;
    //  public static final double kI_launch = 0.0;
    //  public static final double kD_launch = 0.00;

     public static final double kP_arm = 0.025;
     public static final double kI_arm = 0.00;
     public static final double kD_arm = 0.01;
     public static final double kAF_arm = 0.00;

       public static final double positionMax = 75.0;
       public static final double positionUp = 60.0;
       public static final double positionStation = 35.5;
       public static final double positionHorizontal = 0.0;
       public static final double positionReef = -27.0;
       public static final double positionMin = -30.0;
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
  public static final class AlgaeArmConstants
  {
    public static final int angleID = 31;
    public static final int powerLeaderID = 32;
    public static final int powerFollowerID = 33;

    public static final double powerLaunch = 1.0;
    public static final double powerIntake = 0.55;

    public static final int MaxCurrentLimitAngle = 40; // Max current limit of elevator motors 
    public static final int MaxCurrentLimitPower = 100; // Max current limit of elevator motors 

    public static final double maxOutput = 1.0;   // Max power output of elevator motors
    public static final double maxVelocity = 5000.0; // Max velocity of elevator
    public static final double maxAcceleration = 10000.0; // Max acceleration of elevator
    public static final double countsPerDegreePower = 360;   // Encoder ticks per inch of elevator travel
    public static final double countsPerDegreeAngle = 360;   // Encoder ticks per inch of elevator travel
    public static final double posTolerance = 6.0;    // Tolerance for PID control of elevator

    // PID values
    public static final double kP_angle = 0.02;
    public static final double kI_angle = 0.00;
    public static final double kD_angle = 0.01;
    public static final double kAF_angle = 0.02;

    // public static final double kP_power = 0.1;
    // public static final double kI_power = 0.0;
    // public static final double kD_power = 0.001;

    public static final double positionMax = 85.0;
    public static final double positionUp = 83.0;
    public static final double positionProcessor = 4.0;
    public static final double positionReef = -15.0;
    public static final double positionFloor = -35.0;
    public static final double positionMin = -38.0;
    public static final double positionNet = 45.0;
    public static final double positionLollipop = 2.0;
   }

public static final class HangerConstants
{
  public static final int RightHangerID = 40;  // Can Id of Hanger motor right
  public static final int LeftHangerID = 41;  // Can Id of Hanger motor left
  public static final double StartingAngle = 50.0;
  public static final double ReverseAngle = 25.0;
  public static final double HangAngle = 84;
  public static final double HangPower = 1; 
  public static final double HangRampRate = 1.0;
  public static final double PGain = 0.005;
  public static final double maxVelocity = 20000;
  public static final double maxAcceleration = 9000;
  public static final double AllowableError = 2.0;
  public static final double MaxCurrentLimit = 80;
  public static final double DegreesPerRevolution = 360;
  public static final double positionMax = 85;  
  public static final double positionMin = 15;
  public static final double kP = 0.005;
  public static final double kI = 0;
  public static final double kD = 0;
  public static final double kIz = 0;
  public static final double kAF = 0;

}

}
