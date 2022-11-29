package org.firstinspires.ftc.teamcode.opmode.unittest.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.control.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.hardware.commands.FollowTrajectorySequenceCommand;
import org.firstinspires.ftc.teamcode.opmode.OpModeBase;

/*
 * This test shows a sample on how to use a finite state machine.
 *
 * The test alternates between driving forward, stop and backward.
 * Note that the test only schedules at an interval of SCHEDULE_INTERVAL.
 */
@Config
@Autonomous(name = "Autotest", group = "Autonomous")
public class AutoTest extends OpModeBase
{
  public static AutoState state = AutoState.Stop;

  ElapsedTime elapsedTime = new ElapsedTime();

  public static double DRIVE_DISTANCE    = 48; // 48 inches
  public static double STOP_TIME         = 3; // 3 seconds
  public static double SCHEDULE_INTERVAL = 3.0; // 3 seconds

  private int     counter = 1;
  private boolean forward = true;

  @Override
  public void initialize()
  {
    super.initialize();

    // Reset timer
    elapsedTime.reset();
  }

  @Override
  public void update()
  {
    if (elapsedTime.seconds() % SCHEDULE_INTERVAL == 0)
    {
      switch (counter)
      {
        case 0: // Forward
        {
          forward = false;
          counter++;
        }
        break;

        case 1: // Stop
        {
          if (forward)
          {
            counter--;
          } else
          {
            counter++;
          }
        }
        break;

        case 2: // Backward
        {
          forward = true;
          counter--;
        }

        elapsedTime.reset();
      }

      state = autoStateIDtoEnum(counter);
      updateFiniteStateMachine(state);
    }

    telemetry.addData(">", "Current State State: " + state.toString());
  }

  public void updateFiniteStateMachine(AutoState state)
  {
    switch (state)
    {
      case Forward:
      {
        TrajectorySequence forwardSequence = driveEngine.trajectorySequenceBuilder(driveEngine.getPoseEstimate())
            .forward(DRIVE_DISTANCE)
            .build();


        schedule(new InstantCommand(() ->
                                    {
                                      telemetry.addLine("> Scheduled new Forward Sequence.");
                                      telemetry.update();
                                    }),
                 new FollowTrajectorySequenceCommand(driveEngine, forwardSequence));
      }
      break;

      case Backward:
      {
        TrajectorySequence backwardSequence = driveEngine.trajectorySequenceBuilder(driveEngine.getPoseEstimate())
            .waitSeconds(DRIVE_DISTANCE)
            .build();

        schedule(new InstantCommand(() ->
                                    {
                                      telemetry.addLine("> Scheduled new Backward Sequence.");
                                      telemetry.update();
                                    }),
                 new FollowTrajectorySequenceCommand(driveEngine, backwardSequence));
      }
      break;

      case Stop:
      {
        TrajectorySequence waitSequence = driveEngine.trajectorySequenceBuilder(driveEngine.getPoseEstimate())
            .waitSeconds(STOP_TIME)
            .build();

        schedule(new InstantCommand(() ->
                                    {
                                      telemetry.addLine("> Scheduled new Wait Sequence.");
                                      telemetry.update();
                                    }),
                 new FollowTrajectorySequenceCommand(driveEngine, waitSequence));
      }
      break;

      default:
      {
        driveEngine.setMotorPowers(0, 0, 0, 0);
      }
    }
  }

  public AutoState autoStateIDtoEnum(int id)
  {
    switch (id)
    {
      case 0:
        return AutoState.Forward;
      case 1:
        return AutoState.Stop;
      case 2:
        return AutoState.Backward;
      case 3:
        return AutoState.ENUM_COUNT;
    }

    return AutoState.ENUM_COUNT;
  }

  public enum AutoState
  {
    Forward,
    Stop,
    Backward,

    ENUM_COUNT
  }
}
