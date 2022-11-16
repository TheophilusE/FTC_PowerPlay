package org.firstinspires.ftc.teamcode.opmode.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

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

      LiftSubsystem liftSubsystem = new LiftSubsystem(hardwareMap, "liftMotor", "colorDistanceSensor");
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
        // Evaluate Vision state
        if (idleElapsedTime.seconds() > maxIdleTime)
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
        switch (signalState)
        {
          case SIGNAL_NONE:
          {
          }
          break;

          case SIGNAL_ONE:
          {
          }
          break;

          case SIGNAL_TWO:
          {
          }
          break;

          case SIGNAL_THREE:
          {
          }
          break;
        }
      }
      break;

      default:
      {
        driveEngine.setZeroPower();
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
