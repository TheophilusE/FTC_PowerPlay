package org.firstinspires.ftc.teamcode.opmode.unittest.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.teamcode.hardware.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.opmode.OpModeBase;

@Config
@TeleOp(name = "Lift Test", group = "TeleOpUnitTest")
public class LiftTest extends OpModeBase
{
  public static AutoState state = AutoState.ONE_LEVEL;

  @Override
  public void initialize()
  {
    super.initialize();

    addSubsystem(new LiftSubsystem(hardwareMap, "liftMotor", "colorDistanceSensor"));
  }

  @Override
  public void update()
  {
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
