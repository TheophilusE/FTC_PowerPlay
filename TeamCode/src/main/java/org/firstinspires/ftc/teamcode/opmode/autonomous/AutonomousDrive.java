package org.firstinspires.ftc.teamcode.opmode.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.control.derived.AutonomousUtils;
import org.firstinspires.ftc.teamcode.control.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.hardware.commands.FollowTrajectorySequenceCommand;
import org.firstinspires.ftc.teamcode.hardware.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.hardware.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.hardware.subsystems.VisionCVSubsystem;
import org.firstinspires.ftc.teamcode.math.Vector3d;
import org.firstinspires.ftc.teamcode.opmode.Defines;
import org.firstinspires.ftc.teamcode.opmode.OpModeBase;
import org.firstinspires.ftc.teamcode.statemachine.StateMachine;
import org.firstinspires.ftc.teamcode.statemachine.StateMachineConfig;

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
  /// State Machine Manager
  StateMachine<Defines.AutonomousFSMState, Defines.AutonomousFSMTrigger>       m_StateMachine;
  StateMachineConfig<Defines.AutonomousFSMState, Defines.AutonomousFSMTrigger> m_StateMachineConfig;

  boolean initialized = false;

  @Override
  public void initialize()
  {
    super.initialize();

    AutonomousUtils.Initialize();
  }

  @Override
  public void registerSubsystems()
  {
    super.registerSubsystems();

    AutonomousUtils.Initialize();

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
    if (!initialized)
    {
      // Configure autonomous states.
      {
        m_StateMachineConfig = new StateMachineConfig<>();

        // If we are currently idling, switch to the evaluate vision state.
        m_StateMachineConfig.configure(Defines.AutonomousFSMState.IDLE_STATE)
            .onEntry(this::enterIdleState)
            .permit(Defines.AutonomousFSMTrigger.IDLING, Defines.AutonomousFSMState.EVALUATE_VISION);

        // When evaluating vision, if a valid park zone is detected immediately perform park in target
        // zone. If not found, try to get a valid target and switch to default park if not found.
        m_StateMachineConfig.configure(Defines.AutonomousFSMState.EVALUATE_VISION)
            .onEntry(this::evaluateVision)
            .permit(Defines.AutonomousFSMTrigger.NO_PARK_SIGNAL_FOUND, Defines.AutonomousFSMState.DEFAULT_PARK)
            .permit(Defines.AutonomousFSMTrigger.VALID_PARK_SIGNAL_FOUND, Defines.AutonomousFSMState.PARK_IN_SIGNAL_ZONE);

        // If in default park state, schedule the park command in the default park zone.
        m_StateMachineConfig.configure(Defines.AutonomousFSMState.DEFAULT_PARK)
            .onEntry(this::scheduleDefaultPark);

        // If a valid park signal is found, schedule a park command based on the target signal.
        m_StateMachineConfig.configure(Defines.AutonomousFSMState.PARK_IN_SIGNAL_ZONE)
            .onEntry(this::scheduleAutonomousParkSignal);

        m_StateMachine = new StateMachine<>(Defines.autonomousFSMState, m_StateMachineConfig);
        m_StateMachine.fire(Defines.AutonomousFSMTrigger.IDLING);
      }
      initialized = true;
    }
  }

  public void enterIdleState()
  {
    // Ensure that we are not moving in idle state.
    driveEngine.setZeroPower();

    // Perform a scan to ensure that the correct colour is being detected, due to the fact that
    // The park signal is randomized after initialization.
    {
      ElapsedTime waitTime    = new ElapsedTime();
      double      maxWaitTime = 5000; // Milliseconds

      while (waitTime.milliseconds() < maxWaitTime && getVisionState() == Defines.ParkTargetSignal.SIGNAL_NONE)
      {
        // Hold here and wait for the vision subsystem to detect a park signal within the given time
        // In milliseconds. The default park will be used if no signal detected.
      }
    }

    m_StateMachine.fire(Defines.AutonomousFSMTrigger.IDLING);
  }

  public void evaluateVision()
  {
    if (getVisionState() != Defines.ParkTargetSignal.SIGNAL_NONE)
    {
      m_StateMachine.fire(Defines.AutonomousFSMTrigger.VALID_PARK_SIGNAL_FOUND);
    } else
    {
      m_StateMachine.fire(Defines.AutonomousFSMTrigger.NO_PARK_SIGNAL_FOUND);
    }
  }

  // TODO: Set time
  public void scheduleDefaultPark()
  {
    DriveSubsystem driveSubsystem = getComponent(DriveSubsystem.class);
    if (driveSubsystem != null)
    {
      ElapsedTime elapsedTime = new ElapsedTime();
      while (elapsedTime.milliseconds() < 1500)
      {
        if (Defines.BLUE_ALLIANCE)
        {
          driveSubsystem.setMovementVector(new Vector3d(1, 0, 0));
          driveSubsystem.updateMovementStateDifferentialTrigRC();
        } else
        {
          driveSubsystem.setMovementVector(new Vector3d(-1, 0, 0));
        }
      }
      driveSubsystem.setMovementVector(new Vector3d(0, 0, 0));
      driveSubsystem.updateMovementStateDifferentialTrigRC();
    }
  }

  // TODO: Change time
  public void scheduleAutonomousParkSignal()
  {
    DriveSubsystem driveSubsystem = getComponent(DriveSubsystem.class);
    if (driveSubsystem == null)
    {
      return;
    }

    Defines.ParkTargetSignal parkTargetSignal = getVisionState();

    switch (parkTargetSignal)
    {
      /// Park in the leftward zone
      case SIGNAL_ONE:
      {
        ElapsedTime elapsedTime = new ElapsedTime();
        while (elapsedTime.milliseconds() < 1380)
        {
          driveSubsystem.setMovementVector(new Vector3d(-1, 0, 0));
          driveSubsystem.updateMovementStateDifferentialTrigRC();
        }

        elapsedTime.reset();

        while (elapsedTime.milliseconds() < 1300)
        {
          driveSubsystem.setMovementVector(new Vector3d(0, 1, 0));
          driveSubsystem.updateMovementStateDifferentialTrigRC();
        }
        driveSubsystem.setMovementVector(new Vector3d(0, 0, 0));
        driveSubsystem.updateMovementStateDifferentialTrigRC();
      }
      break;

      /// Park in the zone in front of the robot.
      case SIGNAL_TWO:
      {
        ElapsedTime elapsedTime = new ElapsedTime();
        while (elapsedTime.milliseconds() < 1500)
        {
          driveSubsystem.setMovementVector(new Vector3d(0, 1, 0));
          driveSubsystem.updateMovementStateDifferentialTrigRC();
        }
        driveSubsystem.setMovementVector(new Vector3d(0, 0, 0));
        driveSubsystem.updateMovementStateDifferentialTrigRC();
      }
      break;

      /// Park in the zone rightward of the robot.
      case SIGNAL_THREE:
      {
        ElapsedTime elapsedTime = new ElapsedTime();
        while (elapsedTime.milliseconds() < 1050)
        {
          driveSubsystem.setMovementVector(new Vector3d(1, 0, 0));
          driveSubsystem.updateMovementStateDifferentialTrigRC();
        }

        elapsedTime.reset();

        while (elapsedTime.milliseconds() < 1150)
        {
          driveSubsystem.setMovementVector(new Vector3d(0, 1, 0));
          driveSubsystem.updateMovementStateDifferentialTrigRC();
        }
        driveSubsystem.setMovementVector(new Vector3d(0, 0, 0));
        driveSubsystem.updateMovementStateDifferentialTrigRC();
      }
      break;

      default:
      {
        telemetry.addData("> ", parkTargetSignal.toString() + " Is unhandled by the method.");
      }
      break;
    }
  }

  public void scheduleAutonomousTrajectoryDefaultPark()
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

  public void scheduleAutonomousParkSignalTrajectory()
  {
    Defines.ParkTargetSignal parkTargetSignal = getVisionState();

    switch (parkTargetSignal)
    {
      /// Park in the leftward zone
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

      /// Park in the zone in front of the robot.
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

      /// Park in the zone rightward of the robot.
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

      default:
      {
        telemetry.addData("> ", parkTargetSignal.toString() + " Is unhandled by the method.");
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
}
