package org.firstinspires.ftc.teamcode.opmode.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.control.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.hardware.commands.FollowTrajectorySequenceCommand;
import org.firstinspires.ftc.teamcode.hardware.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.hardware.subsystems.VisionCVSubsystem;
import org.firstinspires.ftc.teamcode.opmode.Defines;
import org.firstinspires.ftc.teamcode.opmode.OpModeBase;

/*
 * This class houses the main autonomous program that enables full functionality.
 *
 * The plan for this class is laid out as follows
 * - Start in the IDLE state (will enter to the EVALUATE_VISION state
 * - Evaluate Vision state (deduce which is the target signal)
 * - Park based on a given target signal, If none, do a default park
 */

@Config
@Autonomous(name = "Autonomous Drive", group = "Autonomous")
public class AutonomousDrive extends OpModeBase
{
  /// Timers
  private ElapsedTime idleElapsedTime = new ElapsedTime();

  double maxIdleTime = 5.0;

  @Override
  public void initialize()
  {
    super.initialize();

    if (!Defines.FSM_STATE_OVERRIDE)
    {
      Defines.autonomousFSM = Defines.AutonomousFSM.IDLE;
    }

    idleElapsedTime.reset();
  }

  @Override
  public void registerSubsystems()
  {
    super.registerSubsystems();

    // Register Lift Subsystem
    {
      telemetry.addLine("> Register Lift Subsystem...");

      LiftSubsystem liftSubsystem = new LiftSubsystem(hardwareMap, Defines.LIFT_MOTOR, Defines.COLOR_DISTANCE_SENSOR);
      liftSubsystem.enableTracking = false;
      addSubsystem(liftSubsystem);

      telemetry.update();
    }

    // Register VisionCV Subsystem
    {
      telemetry.addLine("> Register VisionCV Subsystem...");

      addSubsystem(new VisionCVSubsystem(hardwareMap, telemetry));

      telemetry.update();
    }
  }

  @Override
  public void update()
  {
    // Update Finite State Machine
    updateFSM(Defines.autonomousFSM);
  }

  public void updateFSM(Defines.AutonomousFSM state)
  {
    switch (state)
    {
      case IDLE:
      {
        // Evaluate Vision state after the max idle time or we have found a suitable target signal
        if (idleElapsedTime.seconds() > maxIdleTime || getVisionState() != Defines.ParkTargetSignal.SIGNAL_NONE)
        {
          Defines.autonomousFSM = Defines.AutonomousFSM.EVALUATE_VISION;
          return;
        }

        // Ensure that we are not moving in the idle state.
        driveEngine.setZeroPower();
      }
      break;

      case EVALUATE_VISION:
      {
        /// Build trajectory and schedule the command
        Defines.ParkTargetSignal signalState = getVisionState();
        scheduleAutonomousTrajectory(signalState);
      }
      break;
    }
  }

  Defines.ParkTargetSignal getVisionState()
  {
    Defines.ParkTargetSignal result = Defines.ParkTargetSignal.SIGNAL_NONE;

    VisionCVSubsystem visionCVSubsystem = getComponent(VisionCVSubsystem.class);
    if (visionCVSubsystem != null)
    {
      result = visionCVSubsystem.getTargetSignal();
    }

    return result;
  }

  public void scheduleAutonomousTrajectory(Defines.ParkTargetSignal parkTargetSignal)
  {
    switch (parkTargetSignal)
    {
      case SIGNAL_NONE:
      {
        TrajectorySequence defaultPark = null;

        // Build park trajectory
        if (Defines.BLUE_ALLIANCE)
        {
          // A simple strafe to the right will do
          defaultPark = driveEngine.trajectorySequenceBuilder(driveEngine.getPoseEstimate())
              .strafeRight(24 * 4)
              .build();

        } else
        {
          // A simple strafe to the left will do
          defaultPark = driveEngine.trajectorySequenceBuilder(driveEngine.getPoseEstimate())
              .strafeLeft(24 * 4)
              .build();
        }

        // Schedule command
        schedule(new SequentialCommandGroup(
            new FollowTrajectorySequenceCommand(driveEngine, defaultPark)
        ));
      }
      break;

      case SIGNAL_ONE:
      {
        TrajectorySequence defaultPark = null;

        // Build park trajectory, this is Alliance agnostic
        // The zone is always to the left of the robot
        defaultPark = driveEngine.trajectorySequenceBuilder(driveEngine.getPoseEstimate())
            .strafeLeft(24 * 2)
            .forward(24 * 4)
            .build();

        // Schedule command
        schedule(new SequentialCommandGroup(
            new FollowTrajectorySequenceCommand(driveEngine, defaultPark)
        ));
      }
      break;

      case SIGNAL_TWO:
      {
        TrajectorySequence defaultPark = null;

        // Build park trajectory, this is Alliance agnostic
        // A simple strafe forwards will do, the zone is always in front of the robot
        defaultPark = driveEngine.trajectorySequenceBuilder(driveEngine.getPoseEstimate())
            .forward(24 * 4)
            .build();

        // Schedule command
        schedule(new SequentialCommandGroup(
            new FollowTrajectorySequenceCommand(driveEngine, defaultPark)
        ));
      }
      break;

      case SIGNAL_THREE:
      {
        TrajectorySequence defaultPark = null;

        // Build park trajectory, this is Alliance agnostic
        // The zone is always to the right of the robot
        defaultPark = driveEngine.trajectorySequenceBuilder(driveEngine.getPoseEstimate())
            .strafeRight(24 * 2)
            .forward(24 * 4)
            .build();

        // Schedule command
        schedule(new SequentialCommandGroup(
            new FollowTrajectorySequenceCommand(driveEngine, defaultPark)
        ));
      }
      break;
    }
  }
}
