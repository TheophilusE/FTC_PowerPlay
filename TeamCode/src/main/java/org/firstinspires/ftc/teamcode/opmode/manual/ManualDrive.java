package org.firstinspires.ftc.teamcode.opmode.manual;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.control.derived.AutonomousUtils;
import org.firstinspires.ftc.teamcode.hardware.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.hardware.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.hardware.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.math.Vector3d;
import org.firstinspires.ftc.teamcode.opmode.Defines;
import org.firstinspires.ftc.teamcode.opmode.OpModeBase;

/*
 * This class houses the main TeleOp program that enables full functionality.
 */

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

    AutonomousUtils.InitializeHeading();

    // Register Drive Subsystem
    {
      telemetry.addLine("> Register Drive Subsystem...");
      telemetry.update();

      DriveSubsystem driveSubsystem = new DriveSubsystem(driveEngine, telemetry, Defines.DRIVE_MODE);

      addSubsystem(driveSubsystem);
    }

    // Register Lift Subsystem
    {
      telemetry.addLine("> Register Lift Subsystem...");
      telemetry.update();

      LiftSubsystem liftSubsystem = new LiftSubsystem(hardwareMap, Defines.LIFT_MOTOR, Defines.COLOR_DISTANCE_SENSOR);
      liftSubsystem.enableTracking = false;
      liftSubsystem.setTargetPosition(LiftSubsystem.LiftLevel.ZERO_LEVEL);

      addSubsystem(liftSubsystem);
    }

    // Register Claw Subsystem
    {
      telemetry.addLine("> Register Claw Subsystem...");
      telemetry.update();

      ClawSubsystem clawSubsystem = new ClawSubsystem(hardwareMap, Defines.CLAW_MOTORS[0], Defines.CLAW_MOTORS[1], 0.0, 0.35);

      addSubsystem(clawSubsystem);
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
      telemetry.addData(">", " Motors: Left Front Power (%.2f), Right Front Power (%.2f)",
                        driveEngine.getLeftFrontMotor().getPower(), driveEngine.getRightFrontMotor().getPower());
      telemetry.addData(">", " Motors: Left Back Power (%.2f), Right Back Power (%.2f)",
                        driveEngine.getLeftRearMotor().getPower(), driveEngine.getRightRearMotor().getPower());

      {
        LiftSubsystem liftSubsystem = getComponent(LiftSubsystem.class);
        if (liftSubsystem != null)
        {
          telemetry.addData(">", " Current Distance to Floor: %.2f", liftSubsystem.getCurrentDistanceToFloor());
        }
      }
    }
  }

  /*
   * Update translation and rotation of the robot.
   */
  protected void updateMovementState()
  {
    // [-1: Up,   1: Down]
    // [-1: Left, 1: Right]

    // Update field relative offset if using the IMU.
    if (Defines.DRIVE_MODE == Defines.DriveMode.FIELD_CENTRIC_IMU)
    {
      double heading = driveEngine.getHeadingOffset(robotAngleOffset);

      if (gamepad1.left_bumper && gamepad1.a && !previousState)
      {
        robotAngleOffset += heading;
      }

      previousState = gamepad1.left_bumper && gamepad1.a;
    }

    // Supply drive subsystem with human input coefficients
    {
      DriveSubsystem driveSubsystem = getComponent(DriveSubsystem.class);
      if (driveSubsystem != null)
      {
        // Set the current drive mode that may be updated through the dashboard
        driveSubsystem.setDriveMode(Defines.DRIVE_MODE);

        // Set movemement vector from gamepad input.
        driveSubsystem.setMovementVector(new Vector3d(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x));

        // Update heading if using the IMU for the field relative drive mode.
        if (Defines.DRIVE_MODE == Defines.DriveMode.FIELD_CENTRIC_IMU)
        {
          driveSubsystem.setHeading(driveEngine.getHeadingOffset(robotAngleOffset));
        }
      }
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
    if (true)
    {
      LiftSubsystem liftSubsystem = getComponent(LiftSubsystem.class);
      if (liftSubsystem != null)
      {
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

    // Update claw subsystem
    {
      if (gamepad1.b)
      {
        ClawSubsystem clawSubsystem = getComponent(ClawSubsystem.class);
        if (clawSubsystem != null)
        {
          clawSubsystem.setServoPositions(0.0, 0.35);
        }
      }
      if (gamepad1.a)
      {
        ClawSubsystem clawSubsystem = getComponent(ClawSubsystem.class);
        if (clawSubsystem != null)
        {
          clawSubsystem.setServoPositions(0.35, 0.0);
        }
      }
    }
  }
}


