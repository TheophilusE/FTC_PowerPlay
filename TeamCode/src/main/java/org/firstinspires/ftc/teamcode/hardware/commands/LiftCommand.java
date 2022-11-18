package org.firstinspires.ftc.teamcode.hardware.commands;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;

import org.firstinspires.ftc.teamcode.hardware.subsystems.LiftSubsystem;

public class LiftCommand extends ParallelCommandGroup
{
  private final LiftSubsystem           liftSubsystem;
  private final LiftSubsystem.LiftLevel liftLevel;

  public LiftCommand(LiftSubsystem subsystem, LiftSubsystem.LiftLevel liftLevel)
  {
    liftSubsystem  = subsystem;
    this.liftLevel = liftLevel;

    addRequirements(subsystem);
  }

  @Override
  public void initialize()
  {
    liftSubsystem.setTargetPosition(liftLevel);
  }

  @Override
  public boolean isFinished()
  {
    return liftSubsystem.isTargetReached(0.5);
  }

  @Override
  public void end(boolean interrupted)
  {
    // Nothing to do
  }
}
