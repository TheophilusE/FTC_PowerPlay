package org.firstinspires.ftc.teamcode.hardware.commands;

import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.control.derived.DriveEngine;

/*
 * A command that follows a specified Road Runner Trajectory Asynchronously.
 */
public class FollowTrajectoryCommand extends CommandBase
{
  // Drive engine
  private final DriveEngine engine;
  // Trajectory to follow
  private final Trajectory  trajectory;

  // Construct
  public FollowTrajectoryCommand(DriveEngine driveEngine, Trajectory trajectory)
  {
    engine          = driveEngine;
    this.trajectory = trajectory;
  }

  /* Follow Trajectory on initialized. */
  @Override
  public void initialize()
  {
    if (trajectory == null)
    {
      return;
    }
    engine.followTrajectoryAsync(trajectory);
  }

  /* Update engine control loop */
  @Override
  public void execute()
  {
    engine.update();
  }

  /* End when the target trajectory is reached. */
  @Override
  public boolean isFinished()
  {
    return !engine.isBusy() || trajectory == null;
  }
}
