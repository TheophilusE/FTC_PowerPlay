package org.firstinspires.ftc.teamcode.opmode.manual;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.control.util.Extensions;
import org.firstinspires.ftc.teamcode.hardware.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.opmode.Defines;
import org.firstinspires.ftc.teamcode.opmode.OpModeBase;

@Config
@TeleOp(name = "ManualDrive", group = "TeleOp")
public class ManualDrive extends OpModeBase
{
  @Override
  public void registerAccessors()
  {
    super.registerAccessors();
  }

  @Override
  public void registerSubsystems()
  {
    super.registerSubsystems();

    // Register Lift Subsystem
    {
      telemetry.addLine("> Register Lift Subsystem...");

      LiftSubsystem liftSubsystem = new LiftSubsystem(hardwareMap, "liftMotor", "colorDistanceSensor");
      liftSubsystem.enableTracking = false;
      addSubsystem(liftSubsystem);

      telemetry.update();
    }
  }

  @Override
  public void update()
  {
    updateMovementState();

    updateAccessors();

    // Update telemetry
    {
      telemetry.addData(">", " Run Time (s) (%.2f)", getRuntime());
      telemetry.addData(">", " Motors: Left Front Power (%.2f), Right Front Power (%.2f)", driveEngine.getLeftFrontMotor().getPower(), driveEngine.getRightFrontMotor().getPower());
      telemetry.addData(">", " Motors: Left Back Power (%.2f), Right Back Power (%.2f)", driveEngine.getLeftRearMotor().getPower(), driveEngine.getRightRearMotor().getPower());
    }
  }

  /*
   * Update translation and rotation of the robot.
   */
  protected void updateMovementState()
  {
    // [-1: Up,   1: Down]
    // [-1: Left, 1: Right]

    switch (Defines.DRIVE_MODE)
    {
      case ROBOT_CENTRIC_HOLONOMIC:
      {
        double y  = -gamepad1.left_stick_y; // Remember, this is reversed!
        double x  = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
        double rx = gamepad1.right_stick_x;

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

        telemetry.addData(">", " Left Stick  X: (%.2f)", x);
        telemetry.addData(">", " Left Stick  Y: (%.2f)", y);
        telemetry.addData(">", " Right Stick X: (%.2f)", rx);

        driveEngine.setMotorPowers(leftFrontPower, leftRearPower, rightRearPower, rightFrontPower);
      }
      break;

      case ROBOT_CENTRIC_MECANUM:
      {
        double length   = Math.hypot(gamepad1.left_stick_x, -gamepad1.left_stick_y);
        double angle    = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) - (Math.PI * 0.25);
        double rotation = gamepad1.right_stick_x;

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

        // Ensure that the length of the vector is the 1
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

        telemetry.addData(">", " Left Stick  X: (%.2f)", gamepad1.left_stick_x);
        telemetry.addData(">", " Left Stick  Y: (%.2f)", -gamepad1.left_stick_y);
        telemetry.addData(">", " Right Stick X: (%.2f)", gamepad1.right_stick_x);
        telemetry.addData("> ", " Length:   (%.2f)", length);
        telemetry.addData("> ", " Angle:    (%.2f)", angle);
        telemetry.addData("> ", " Rotation: (%.2f)", rotation);

        driveEngine.setMotorPowers(leftFrontPower, leftRearPower, rightRearPower, rightFrontPower);
      }
      break;

      case FIELD_CENTRIC_GAMEPAD:
      {
        // Get current position estimate
        Pose2d currentPosition = driveEngine.getPoseEstimate();

        // Get vector direction, derived from the (x, y) positions on the gamepad control axes
        // And rotate the vector by the inverse of the current heading
        Vector2d inputVector = new Vector2d(-gamepad1.left_stick_x, gamepad1.left_stick_y).rotated(-currentPosition.getHeading());

        // Pass in the rotated input + right stick value for rotation
        // Rotation is not part of the rotated input, thus, it must be passed in separately
        driveEngine.setWeightedDrivePower(new Pose2d(
            inputVector.getX() * Defines.DRIVE_COEFFICIENT,
            inputVector.getY() * Defines.DRIVE_COEFFICIENT,
            -gamepad1.right_stick_x * Defines.DRIVE_COEFFICIENT
        ));

        telemetry.addData("> Pos X", currentPosition.getX());
        telemetry.addData("> Pos Y", currentPosition.getY());
        telemetry.addData("> Heading", currentPosition.getHeading());
      }
      break;

      case FIELD_CENTRIC_IMU:
      {
        double heading = driveEngine.getHeadingOffset(robotAngleOffset);

        if (gamepad1.left_bumper && gamepad1.a && !previousState)
        {
          robotAngleOffset += heading;
        }

        previousState = gamepad1.left_bumper && gamepad1.a;

        double y  = (Math.abs(gamepad1.left_stick_y) > 0.05) ? Extensions.cubeInput(-gamepad1.left_stick_y, 0.4) : 0.0; // Remember, this is reversed!
        double x  = (Math.abs(gamepad1.left_stick_x) > 0.05) ? Extensions.cubeInput(gamepad1.left_stick_x * 1.1, 0.4) : 0.0; // Counteract imperfect strafing
        double rx = (Math.abs(gamepad1.right_stick_x) > 0.05) ? Extensions.cubeInput(gamepad1.right_stick_x, 0.4) : 0.0;

        // Get the field centric inputs
        Pose2d pose = Extensions.toFieldRelative(new Pose2d(x, y, rx), heading);

        x  = pose.getX();
        y  = pose.getY();
        rx = pose.getHeading();

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio, but only when
        // at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

        double frontLeftPower  = (y + x + rx) / denominator;
        double backLeftPower   = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower  = (y + x - rx) / denominator;

        driveEngine.setMotorPowers(
            frontLeftPower * Defines.DRIVE_COEFFICIENT,
            backLeftPower * Defines.DRIVE_COEFFICIENT,
            backRightPower * Defines.DRIVE_COEFFICIENT,
            frontRightPower * Defines.DRIVE_COEFFICIENT
                                  );

        telemetry.addLine("> Press the left bumper to re-zero the heading.");
        telemetry.addData("> Current Heading with offset", AngleUnit.DEGREES.fromRadians(driveEngine.getHeadingOffset(robotAngleOffset)));
        telemetry.addData("> Offset", AngleUnit.DEGREES.fromRadians(robotAngleOffset));
      }
      break;

      default:
        break;
    }
  }

  /*
   * Update Accessors.
   */
  public void updateAccessors()
  {
    // Update drive coefficient through the Gamepad.
    {
      // Speed limiter accessor update
      if (!gamepad1.left_bumper)
      {
        if (gamepad1.dpad_up)
        {
          Defines.DRIVE_COEFFICIENT = 1.0f;
        } else if (gamepad1.dpad_right)
        {
          Defines.DRIVE_COEFFICIENT = 0.75f;
        } else if (gamepad1.dpad_down)
        {
          Defines.DRIVE_COEFFICIENT = 0.5f;
        } else if (gamepad1.dpad_left)
        {
          Defines.DRIVE_COEFFICIENT = 0.25f;
        }
      } else
      {
        if (gamepad1.dpad_up)
        {
          Defines.DRIVE_COEFFICIENT = (0.5f + 0.25f + 0.75f + 1.0f) / 2.0f;
        } else if (gamepad1.dpad_right)
        {
          Defines.DRIVE_COEFFICIENT = (0.5f + 0.25f + 0.75f) / 2.0f;
        } else if (gamepad1.dpad_down)
        {
          Defines.DRIVE_COEFFICIENT = (0.5f + 0.25f) / 2.0f;
        } else if (gamepad1.dpad_left)
        {
          Defines.DRIVE_COEFFICIENT = 0.25f / 2.0f;
        }
      }
    }

    // Update Lift Subsystem
    {
      LiftSubsystem liftSubsystem = getComponent(LiftSubsystem.class);
      if (liftSubsystem != null)
      {
        liftSubsystem.setTargetPosition(LiftSubsystem.LiftLevel.ZERO_LEVEL);

        // This is a less hacky method as it only applies power as the difference
        // between the up and down vectors.
        // If the right trigger is greater than the left trigger, it'll go up. (Ex. 1 - 0 = 1)
        // If the left trigger is greater than the right trigger, it'll go down. (Ex. 0 - 1 = -1)
        // If they are the same, it will apply zero power (Ex. 1 - 1 = 0 or 0 - 0 = 0)
        double difference = gamepad1.right_trigger - gamepad1.left_trigger;
        liftSubsystem.getLiftMotor().setPower(difference);
      }
    }

    // Update Lift Subsystem
    if (false) // Disabled for now
    {
      // Rest Position (zero or start position)
      if (gamepad1.x)
      {
        LiftSubsystem liftSubsystem = getComponent(LiftSubsystem.class);
        if (liftSubsystem != null)
        {
          liftSubsystem.setTargetPosition(LiftSubsystem.LiftLevel.THREE_LEVEL);
        }
      }

      // First position (smallest position)
      if (gamepad1.a)
      {
        LiftSubsystem liftSubsystem = getComponent(LiftSubsystem.class);
        if (liftSubsystem != null)
        {
          liftSubsystem.setTargetPosition(LiftSubsystem.LiftLevel.ZERO_LEVEL);
        }
      }

      // Second position (middle position)
      if (gamepad1.b)
      {
        LiftSubsystem liftSubsystem = getComponent(LiftSubsystem.class);
        if (liftSubsystem != null)
        {
          liftSubsystem.setTargetPosition(LiftSubsystem.LiftLevel.ONE_LEVEL);
        }
      }

      // Third Position (highest position)
      if (gamepad1.y)
      {
        LiftSubsystem liftSubsystem = getComponent(LiftSubsystem.class);
        if (liftSubsystem != null)
        {
          liftSubsystem.setTargetPosition(LiftSubsystem.LiftLevel.TWO_LEVEL);
        }
      }
    }
  }
}
