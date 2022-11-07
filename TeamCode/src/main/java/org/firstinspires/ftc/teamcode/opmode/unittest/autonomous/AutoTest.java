package org.firstinspires.ftc.teamcode.opmode.unittest.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmode.OpModeBase;

@Config
@Autonomous(name = "Autotest", group = "Autonomous")
public class AutoTest extends OpModeBase
{
  public static AutoState state = AutoState.Stop;

  @Override
  public void update()
  {
    switch (state)
    {
      case Forward:
      {
        driveEngine.setMotorPowers(1, 1, 1, 1);
      }
      break;

      case Stop:
      {
        driveEngine.setMotorPowers(0, 0, 0, 0);
      }
      break;

      default:
      {
        driveEngine.setMotorPowers(0, 0, 0, 0);
      }
    }
  }

  public enum AutoState
  {
    Forward,
    Stop
  }
}
