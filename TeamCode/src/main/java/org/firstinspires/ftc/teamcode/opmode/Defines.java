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
  public static boolean BLUE_ALLIANCE = true;

  /// Start Position. This defines the robot's initial heading and location.
  public static Pose2d START_POSE = new Pose2d(6, BLUE_ALLIANCE ? 63 : -63,
                                               Math.toRadians(BLUE_ALLIANCE ? -90 : 90));

  /// Drive Coefficient. This defines the scale factor [0.0, 1.0] of which to scale the drive
  /// Output Power.
  public static float DRIVE_COEFFICIENT = 1.0f;

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

    ENUM_COUNT
  }

  /// Drive mode configuration. Publicly accessible variable to access current drive mode config.
  public static DriveMode DRIVE_MODE = DriveMode.ROBOT_CENTRIC_MECANUM;

  /*
   * Enumeration of all states used used for autonomous logic and processing.
   *
   * This is a Finite State Machine (FSM) that can be used to procure dynamic autonomous logic,
   * where it's behaviour is based on what state the controller is.
   */
  public enum AutonomousFSM
  {
    /// Idle state in the controller.
    IDLE,

    /// Indicates that the controller should move to a position.
    MOVE_TO_POSITION,

    /// Indicates that the controller should align its heading.
    ALIGN_HEADING,

    /// Indicates that the controller should process logic through its vision subsystem.
    EVALUATE_VISION,

    ENUM_COUNT
  }

  /// Specifies if autonomous programs should use their default state or start at a set state.
  public static boolean FSM_STATE_OVERRIDE = false;

  /// Stores the global autonomous state.
  public static AutonomousFSM autonomousFSM = AutonomousFSM.IDLE;

  /// Enable / Disable Camera Stream to the Dashboard.
  public static boolean ENABLE_CAMERA_STREAM = true;

  /// Specify the number of frames a second to render onto the Dashboard.
  public static int STREAM_MAX_FPS = 10;

  /*
   * Enumeration of the park signals, wherein the robot should park in the Autonomous period.
   */
  public enum ParkTargetSignal
  {
    /// No park signal given.
    SIGNAL_NONE,

    /// Leftmost park target.
    SIGNAL_ONE,

    /// Middle park target.
    SIGNAL_TWO,

    /// Rightmost park target.
    SIGNAL_THREE,

    ENUM_COUNT
  }
}
