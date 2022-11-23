package org.firstinspires.ftc.teamcode.opmode.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.control.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.hardware.commands.ClawCommand;
import org.firstinspires.ftc.teamcode.hardware.commands.FollowTrajectorySequenceCommand;
import org.firstinspires.ftc.teamcode.hardware.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.opmode.Defines;
import org.firstinspires.ftc.teamcode.opmode.OpModeBase;

@Config
@Autonomous(name = "AutoPark", group = "Autonomous")
public class AutoPark extends OpModeBase
{


  @Override
  public void initialize()
  {
    super.initialize();

    if (!Defines.FSM_STATE_OVERRIDE)
    {
      Defines.autonomousFSM = Defines.AutonomousFSM.IDLE;
    }

    addSubsystem(new ClawSubsystem(hardwareMap));

    TrajectorySequence defaultPark = null;

    // Build park trajectory
    if (Defines.BLUE_ALLIANCE)
    {
      // A simple strafe to the right will do
      defaultPark = driveEngine.trajectorySequenceBuilder(driveEngine.getPoseEstimate())
          .strafeRight(60)
          .forward(60)
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
            new ClawCommand(getComponent(ClawSubsystem.class), 0.0, 0.0),
            new FollowTrajectorySequenceCommand(driveEngine, defaultPark)
        ));

  }

  @Override
  public void update()
  {
    // Update Finite State Machine
    //updateFSM();
  }

  public void updateFSM()
  {
    switch (Defines.autonomousFSM)
    {
      default:
      {
        driveEngine.setZeroPower();
      }
      break;
    }
  }
}