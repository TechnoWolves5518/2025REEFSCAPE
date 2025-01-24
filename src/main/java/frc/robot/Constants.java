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
  public static final double MAX_SPEED  = Units.feetToMeters(14.5);
  public static final Integer FRONT_RIGHT = 4;
  public static final Integer FRONT_LEFT = 10;
  public static final Integer BACK_RIGHT = 1;
  public static final Integer BACK_LEFT = 7;
  public static final double CLIMB_SPEED  = 0.25;
  public static final double MANIPULATE_SPEED  = 0.25;
  // Maximum speed of the robot in meters per second, used to limit acceleration.

//  public static final class AutonConstants
//  {
//
//    public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
//    public static final PIDConstants ANGLE_PID       = new PIDConstants(0.4, 0, 0.01);
//  }

  // CAN IDs
  public static final int CLIMBER = 17;
  public static final int MANIPULATORLEFT = 15;
  public static final int MANIPULATORRIGHT = 16;
  public static final int ELEVATOR = 19;
  public static final int ELEVATOR_ENCODER = 20;


  // Elevator properties
  public static final int ELEVATORSPEED = 1;
  public static final int L1ANGLE = 0;
  public static final int L2ANGLE = 0;
  public static final int L3ANGLE = 0;
  public static final int L4ANGLE = 0;
  public static final int BARGEANGLE = 0;

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
    public static final double LEFT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT    = 6;
  }
}
