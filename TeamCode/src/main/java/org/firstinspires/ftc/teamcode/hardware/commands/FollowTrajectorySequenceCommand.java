package org.firstinspires.ftc.teamcode.hardware.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.control.derived.DriveEngine;
import org.firstinspires.ftc.teamcode.control.trajectorysequence.TrajectorySequence;

/*
 * A command that follows a specified Road Runner Trajectory Sequence Asynchronously.
 */
public class FollowTrajectorySequenceCommand extends CommandBase
{
  // Drive engine
  private final DriveEngine        engine;
  // Trajectory Sequence to follow
  private final TrajectorySequence trajectorySequence;

  // Construct
  public FollowTrajectorySequenceCommand(DriveEngine driveEngine, TrajectorySequence trajectorySequence)
  {
    engine                  = driveEngine;
    this.trajectorySequence = trajectorySequence;
  }

  /* Follow Trajectory Sequence on initialized. */
  @Override
  public void initialize()
  {
    if (trajectorySequence == null)
    {
      return;
    }
    engine.followTrajectorySequenceAsync(trajectorySequence);
  }

  /* Update engine control loop */
  @Override
  public void execute()
  {
    engine.update();
  }

  /* End when the target trajectory sequence is complete. */
  @Override
  public boolean isFinished()
  {
    return !engine.isBusy() || trajectorySequence == null;
  }
}
