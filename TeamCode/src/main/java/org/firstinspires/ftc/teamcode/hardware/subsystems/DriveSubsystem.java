package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.control.derived.DriveEngine;
import org.firstinspires.ftc.teamcode.control.util.Extensions;
import org.firstinspires.ftc.teamcode.math.Vector3d;
import org.firstinspires.ftc.teamcode.opmode.Defines;

/*
 * A subsystem tha houses the movement functionality of the Robot.
 */
public class DriveSubsystem extends SubsystemBase
{
  /// Engine
  protected DriveEngine m_driveEngine = null;

  /// Data
  protected Defines.DriveMode m_driveMode;
  protected Vector3d          m_movementVector;
  protected Telemetry         m_telemetry;
  protected double            m_robotAngleOffset;

  // Construct
  public DriveSubsystem(DriveEngine driveEngine, Telemetry telemetry)
  {
    m_driveEngine      = driveEngine;
    m_driveMode        = Defines.DriveMode.NONE;
    m_telemetry        = telemetry;
    m_movementVector   = new Vector3d();
    m_robotAngleOffset = 0.0;
  }

  public DriveSubsystem(DriveEngine driveEngine, Telemetry telemetry, Defines.DriveMode driveMode)
  {
    m_driveEngine      = driveEngine;
    m_driveMode        = driveMode;
    m_telemetry        = telemetry;
    m_movementVector   = new Vector3d();
    m_robotAngleOffset = 0.0;
  }

  @Override
  public void periodic()
  {
    switch (m_driveMode)
    {
      case ROBOT_CENTRIC_HOLONOMIC:
      {
        updateMovementStateDifferentialRC();
      }
      break;
      case ROBOT_CENTRIC_MECANUM:
      {
        updateMovementStateDifferentialTrigRC();
      }
      break;
      case FIELD_CENTRIC_GAMEPAD:
      {
        updateMovementStateGamepadFC();
      }
      break;
      case FIELD_CENTRIC_IMU:
      {
        updateMovementStateIMUFC();
      }
      break;
      default:
      {
        m_telemetry.addLine("> Unknown or unimplemented drive mode selected '" + m_driveMode.toString() + "'");
        m_driveEngine.setZeroPower();
      }
      break;
    }
  }

  public void setDriveMode(Defines.DriveMode driveMode)
  {
    if (m_driveMode != driveMode)
    {
      m_driveMode = driveMode;
    }
  }

  public void setMovementVector(Vector3d movementVector)
  {
    m_movementVector = movementVector;
  }

  public void setHeading(double heading)
  {
    m_robotAngleOffset = heading;
  }

  public void updateMovementStateDifferentialRC()
  {
    double y  = m_movementVector.y;
    double x  = m_movementVector.x * 1.1; // Counteract imperfect strafing
    double rx = m_movementVector.z;

    // Denominator is the largest motor power (absolute value) or 1
    // This ensures all the powers maintain the same ratio, but only when
    // at least one is out of the range [-1, 1]
    double denominator     = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
    double leftFrontPower  = (y + x + rx) / denominator;
    double leftRearPower   = (y - x + rx) / denominator;
    double rightFrontPower = (y - x - rx) / denominator;
    double rightRearPower  = (y + x - rx) / denominator;

    // Apply driver scaling
    leftFrontPower *= Defines.DRIVE_COEFFICIENT;
    leftRearPower *= Defines.DRIVE_COEFFICIENT;
    rightRearPower *= Defines.DRIVE_COEFFICIENT;
    rightFrontPower *= Defines.DRIVE_COEFFICIENT;

    m_telemetry.addData(">", " Left Stick  X: (%.2f)", x);
    m_telemetry.addData(">", " Left Stick  Y: (%.2f)", y);
    m_telemetry.addData(">", " Right Stick X: (%.2f)", rx);

    m_driveEngine.setMotorPowers(leftFrontPower, leftRearPower, rightRearPower, rightFrontPower);
  }

  public void updateMovementStateDifferentialTrigRC()
  {
    double length   = Math.hypot(m_movementVector.x, m_movementVector.y);
    double angle    = Math.atan2(m_movementVector.y, m_movementVector.x) - (Math.PI * 0.25);
    double rotation = m_movementVector.z;

    double leftFrontPower  = length * Math.cos(angle);
    double leftRearPower   = length * Math.sin(angle);
    double rightRearPower  = length * Math.cos(angle);
    double rightFrontPower = length * Math.sin(angle);

    // If the length of the angle is greater than 1, then we must restrict all power values such
    // that the length is 1 as defined by the Unit Circle.
    if (length > 1.0)
    {
      leftFrontPower /= length;
      leftRearPower /= length;
      rightRearPower /= length;
      rightFrontPower /= length;
    }

    // Apply Rotation
    leftFrontPower += rotation;
    leftRearPower += rotation;
    rightRearPower -= rotation;
    rightFrontPower -= rotation;

    // Ensure that the length of the vector is less than or equal to 1
    double rotLength = Math.hypot(leftFrontPower, rightFrontPower);
    leftFrontPower /= rotLength;
    leftRearPower /= rotLength;
    rightRearPower /= rotLength;
    rightFrontPower /= rotLength;

    // Apply driver scaling
    leftFrontPower *= Defines.DRIVE_COEFFICIENT;
    leftRearPower *= Defines.DRIVE_COEFFICIENT;
    rightRearPower *= Defines.DRIVE_COEFFICIENT;
    rightFrontPower *= Defines.DRIVE_COEFFICIENT;

    m_telemetry.addData(">", " Left Stick  X: (%.2f)", m_movementVector.x);
    m_telemetry.addData(">", " Left Stick  Y: (%.2f)", m_movementVector.y);
    m_telemetry.addData(">", " Right Stick X: (%.2f)", m_movementVector.z);
    m_telemetry.addData("> ", " Length:   (%.2f)", length);
    m_telemetry.addData("> ", " Angle:    (%.2f)", angle);
    m_telemetry.addData("> ", " Rotation: (%.2f)", rotation);

    m_driveEngine.setMotorPowers(leftFrontPower, leftRearPower, rightRearPower, rightFrontPower);
  }

  public void updateMovementStateGamepadFC()
  {
    // Get current position estimate
    Pose2d currentPosition = m_driveEngine.getPoseEstimate();

    // Get vector direction, derived from the (x, y) positions on the gamepad control axes
    // And rotate the vector by the inverse of the current heading
    Vector2d inputVector = new Vector2d(-m_movementVector.x, -m_movementVector.y).rotated(-currentPosition.getHeading());

    // Pass in the rotated input + right stick value for rotation
    // Rotation is not part of the rotated input, thus, it must be passed in separately
    m_driveEngine.setWeightedDrivePower(new Pose2d(
        inputVector.getX() * Defines.DRIVE_COEFFICIENT,
        inputVector.getY() * Defines.DRIVE_COEFFICIENT,
        -m_movementVector.z * Defines.DRIVE_COEFFICIENT
    ));

    m_telemetry.addData("> Pos X", currentPosition.getX());
    m_telemetry.addData("> Pos Y", currentPosition.getY());
    m_telemetry.addData("> Heading", currentPosition.getHeading());
  }

  public void updateMovementStateIMUFC()
  {
    double heading = m_driveEngine.getHeadingOffset(m_robotAngleOffset);

    double y  = (Math.abs(m_movementVector.y) > 0.05) ? Extensions.cubeInput(m_movementVector.y, 0.4) : 0.0;
    double x  = (Math.abs(m_movementVector.x) > 0.05) ? Extensions.cubeInput(m_movementVector.x, 0.4) : 0.0;
    double rx = (Math.abs(m_movementVector.z) > 0.05) ? Extensions.cubeInput(m_movementVector.z, 0.4) : 0.0;

    // Compute the field centric inputs
    Pose2d pose = Extensions.toFieldRelative(new Pose2d(x, y, rx), heading);

    // Set the new movement vector
    x  = pose.getX();
    y  = pose.getY();
    rx = pose.getHeading();

    double length   = Math.hypot(x, y);
    double angle    = Math.atan2(y, x) - (Math.PI * 0.25);
    double rotation = rx;

    double leftFrontPower  = length * Math.cos(angle);
    double leftRearPower   = length * Math.sin(angle);
    double rightRearPower  = length * Math.cos(angle);
    double rightFrontPower = length * Math.sin(angle);

    // If the length of the angle is greater than 1, then we must restrict all power values such
    // that the length is 1 as defined by the Unit Circle.
    if (length > 1.0)
    {
      leftFrontPower /= length;
      leftRearPower /= length;
      rightRearPower /= length;
      rightFrontPower /= length;
    }

    // Apply Rotation
    leftFrontPower += rotation;
    leftRearPower += rotation;
    rightRearPower -= rotation;
    rightFrontPower -= rotation;

    // Ensure that the length of the vector is less than or equal to 1
    double rotLength = Math.hypot(leftFrontPower, rightFrontPower);
    leftFrontPower /= rotLength;
    leftRearPower /= rotLength;
    rightRearPower /= rotLength;
    rightFrontPower /= rotLength;

    // Apply driver scaling
    leftFrontPower *= Defines.DRIVE_COEFFICIENT;
    leftRearPower *= Defines.DRIVE_COEFFICIENT;
    rightRearPower *= Defines.DRIVE_COEFFICIENT;
    rightFrontPower *= Defines.DRIVE_COEFFICIENT;

    m_driveEngine.setMotorPowers(leftFrontPower, leftRearPower, rightRearPower, rightFrontPower);

    m_telemetry.addData(">", " Left Stick  X: (%.2f)", m_movementVector.x);
    m_telemetry.addData(">", " Left Stick  Y: (%.2f)", m_movementVector.y);
    m_telemetry.addData(">", " Right Stick X: (%.2f)", m_movementVector.z);
    m_telemetry.addData("> ", " Length:   (%.2f)", length);
    m_telemetry.addData("> ", " Angle:    (%.2f)", angle);
    m_telemetry.addData("> ", " Rotation: (%.2f)", rotation);

    m_telemetry.addLine("> Press the left bumper to re-zero the heading.");
    m_telemetry.addData("> Current Heading with offset", AngleUnit.DEGREES.fromRadians(m_driveEngine.getHeadingOffset(m_robotAngleOffset)));
    m_telemetry.addData("> Offset", AngleUnit.DEGREES.fromRadians(m_robotAngleOffset));
  }

  public double getCurrentHeadingOffset()
  {
    return m_robotAngleOffset;
  }
}
