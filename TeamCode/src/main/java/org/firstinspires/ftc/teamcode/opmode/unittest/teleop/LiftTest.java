package org.firstinspires.ftc.teamcode.opmode.unittest.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.opmode.OpModeBase;

@Config
@TeleOp(name = "Lift Test", group = "TeleOpUnitTest")
public class LiftTest extends OpModeBase
{
  public static AutoState state            = AutoState.ONE_LEVEL;
  public        int       currentLiftLevel = 0;

  @Override
  public void initialize()
  {
    super.initialize();

    // Register Lift Subsystem
    {
      telemetry.addLine("> Register Lift Subsystem...");

      LiftSubsystem liftSubsystem = new LiftSubsystem(hardwareMap, "liftMotor", "colorDistanceSensor");
      liftSubsystem.enableTracking = true;
      addSubsystem(liftSubsystem);

      telemetry.update();
    }
  }

  @Override
  public void update()
  {
    if (gamepad1.y)
    {
      state = AutoState.THREE_LEVEL;
    }

    if (gamepad1.b)
    {
      state = AutoState.TWO_LEVEL;
    }

    if (gamepad1.a)
    {
      state = AutoState.ONE_LEVEL;
    }

    if (gamepad1.x)
    {
      state = AutoState.ZERO_LEVEL;
    }

    switch (state)
    {
      case ZERO_LEVEL:
      {
        LiftSubsystem liftSubsystem = getComponent(LiftSubsystem.class);
        liftSubsystem.setTargetPosition(LiftSubsystem.LiftLevel.ZERO_LEVEL);
      }
      break;

      case ONE_LEVEL:
      {
        LiftSubsystem liftSubsystem = getComponent(LiftSubsystem.class);
        liftSubsystem.setTargetPosition(LiftSubsystem.LiftLevel.ONE_LEVEL);
      }
      break;

      case TWO_LEVEL:
      {
        LiftSubsystem liftSubsystem = getComponent(LiftSubsystem.class);
        liftSubsystem.setTargetPosition(LiftSubsystem.LiftLevel.TWO_LEVEL);
      }
      break;

      case THREE_LEVEL:
      {
        LiftSubsystem liftSubsystem = getComponent(LiftSubsystem.class);
        liftSubsystem.setTargetPosition(LiftSubsystem.LiftLevel.THREE_LEVEL);
      }
      break;
    }

    driveEngine.setZeroPower();
  }

  public enum AutoState
  {
    ZERO_LEVEL,
    ONE_LEVEL,
    TWO_LEVEL,
    THREE_LEVEL
  }
}
