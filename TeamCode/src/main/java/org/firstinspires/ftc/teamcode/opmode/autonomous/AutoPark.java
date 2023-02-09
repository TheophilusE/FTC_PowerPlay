package org.firstinspires.ftc.teamcode.opmode.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.control.derived.AutonomousUtils;
import org.firstinspires.ftc.teamcode.control.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.hardware.commands.ClawCommand;
import org.firstinspires.ftc.teamcode.hardware.commands.FollowTrajectorySequenceCommand;
import org.firstinspires.ftc.teamcode.hardware.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.hardware.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.opmode.Defines;
import org.firstinspires.ftc.teamcode.opmode.OpModeBase;

@Config
@Autonomous(name = "AutoPark", group = "Autonomous")
public class AutoPark extends OpModeBase
{
  @Override
  public void registerSubsystems()
  {
    super.registerSubsystems();

    AutonomousUtils.Initialize();

    // Register Lift Subsystem
    {
      telemetry.addLine("> Register Lift Subsystem...");

      LiftSubsystem liftSubsystem = new LiftSubsystem(hardwareMap, Defines.LIFT_MOTOR, Defines.COLOR_DISTANCE_SENSOR);
      liftSubsystem.enableTracking = true;
      addSubsystem(liftSubsystem);

      telemetry.update();
    }

    // Register Claw Subsystem
    {
      telemetry.addLine("> Register Claw Subsystem...");

      addSubsystem(new ClawSubsystem(hardwareMap, Defines.CLAW_MOTORS[0], Defines.CLAW_MOTORS[1]));

      telemetry.update();
    }
  }

  @Override
  public void initialize()
  {
    super.initialize();

    if (!Defines.FSM_STATE_OVERRIDE)
    {
      Defines.autonomousFSMState = Defines.AutonomousFSMState.IDLE_STATE;
    }

    // Schedule park command
    {
      TrajectorySequence defaultPark = null;

      // Build park trajectory
      if (Defines.BLUE_ALLIANCE)
      {
        // A simple strafe to the right will do
        defaultPark = driveEngine.trajectorySequenceBuilder(driveEngine.getPoseEstimate())
            .strafeRight(60)
            .build();

      } else
      {
        // A simple strafe to the left will do
        defaultPark = driveEngine.trajectorySequenceBuilder(driveEngine.getPoseEstimate())
            .strafeLeft(60)
            .build();
      }
      // Schedule command
      schedule(
          new SequentialCommandGroup(
              new ClawCommand(getComponent(ClawSubsystem.class), ClawSubsystem.ClawPosition.CLOSE),
              new FollowTrajectorySequenceCommand(driveEngine, defaultPark)
          ));
    }
  }

  @Override
  public void update()
  {
    // Update Finite State Machine
  }
}