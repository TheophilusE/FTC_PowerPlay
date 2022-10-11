package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;

/*
 * Class that holds all all configuration values that drive robot function.
 */
@Config
public final class Defines
{
  /*
   * Override base constructor.
   */
  private Defines()
  {
  }

  /// Define to specify which side the robot starts from. Useful for implementing logic that is
  /// Dependent on the side of the robot.
  public static boolean BLUE_SIDE = false;

  /// Start Pose. This defines the robot's initial heading and location.
  public static Pose2d START_POSE = new Pose2d(6, BLUE_SIDE ? 63 : -63,
                                               Math.toRadians(BLUE_SIDE ? -90 : 90));

  /// Drive Coefficient. This defines the scale factor [0.0, 1.0] of which to scale the drive
  /// Output Power.
  public static float DRIVE_COEFFICIENT = 0.5f;

  /*
   * Enumeration of all supported drive modes. This defines the way the robot is handled.
   *
   * Robot Centric drive modes mean that the input of the user is always relative to the robot.
   *
   * Field Centric drive modes mean that the input of the user is is applied irrespective of the
   * robot's current heading based on a set reference point.
   */
  public enum DriveMode
  {
    ROBOT_CENTRIC_HOLONOMIC,
    ROBOT_CENTRIC_MECANUM,
    FIELD_CENTRIC_GAMEPAD,
    FIELD_CENTRIC_IMU,
  }

  /// Drive mode configuration. Publicly accessible variable to access current drive mode config.
  public static DriveMode DRIVE_MODE = DriveMode.ROBOT_CENTRIC_MECANUM;

  /// Enable / Disable Camera Stream to the Dashboard.
  public static boolean ENABLE_CAMERA_STREAM = true;
}
