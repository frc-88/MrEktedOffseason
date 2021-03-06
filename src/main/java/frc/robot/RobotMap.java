/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
  // For example to map the left and right motors, you could define the
  // following variables to use with your drivetrain subsystem.
  // public static int leftMotor = 1;
  // public static int rightMotor = 2;

  // If you are using multiple modules, make sure to define both the port
  // number and the module. For example you with a rangefinder:
  // public static int rangefinderPort = 1;
  // public static int rangefinderModule = 1;

  public static final int LEFT_MASTER_DRIVE_ID = 31;
  public static final int RIGHT_MASTER_DRIVE_ID = 32;
  public static final int LEFT_FOLLOWER00_DRIVE_ID = 26;
  public static final int LEFT_FOLLOWER01_DRIVE_ID = 24;
  public static final int LEFT_FOLLOWER02_DRIVE_ID = 25;
  public static final int RIGHT_FOLLOWER00_DRIVE_ID = 22;
  public static final int RIGHT_FOLLOWER01_DRIVE_ID = 23;
  public static final int RIGHT_FOLLOWER02_DRIVE_ID = 21;
  
  public static final int DRIVETRAIN_SCALING_EXPONENT = 2;
  public static final int DRIVETRAIN_TURN_SCALING_EXPONENT = 3;
  public static final double DRIVETRAIN_SPEED_DEADBAND = .083;
  public static final double DRIVETRAIN_TURN_DEADBAND = .133;

  public static final double DRIVE_VOLTAGE_LIMIT = 0.83;

  public static final int CAN_TIMEOUT = 1; //in milliseconds

  public static final int MAX_SPEED = 14500;


  public static final double LEFT_DRIVE_KF = .059;
  public static final double RIGHT_DRIVE_KF = .0565;
}
