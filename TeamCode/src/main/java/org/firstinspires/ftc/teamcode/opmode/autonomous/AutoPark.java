package org.firstinspires.ftc.teamcode.opmode.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.control.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.hardware.commands.FollowTrajectoryCommand;
import org.firstinspires.ftc.teamcode.hardware.commands.FollowTrajectorySequenceCommand;
import org.firstinspires.ftc.teamcode.opmode.Defines;
import org.firstinspires.ftc.teamcode.opmode.OpModeBase;

@Config
@Autonomous(name = "AutoPark", group = "Autonomous")
public class AutoPark extends OpModeBase
{
  TrajectorySequence goToParkZone = null;

  @Override
  public void initialize()
  {
    super.initialize();

    if (!Defines.FSM_STATE_OVERRIDE)
    {
      Defines.autonomousFSM = Defines.AutonomousFSM.IDLE;
    }
    
    goToParkZone = driveEngine.trajectorySequenceBuilder(driveEngine.getPoseEstimate())
        .forward(50)
        .turn(3)
        .forward(40)
        .build();

    schedule(new SequentialCommandGroup(
        new FollowTrajectorySequenceCommand(driveEngine, goToParkZone)
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