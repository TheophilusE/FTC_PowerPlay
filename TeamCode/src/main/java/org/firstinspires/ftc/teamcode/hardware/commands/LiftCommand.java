package org.firstinspires.ftc.teamcode.hardware.commands;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;

import org.firstinspires.ftc.teamcode.hardware.subsystems.LiftSubsystem;

public class LiftCommand extends ParallelCommandGroup
{
  private final LiftSubsystem liftSubsystem;

  public LiftCommand(LiftSubsystem subsystem)
  {
    liftSubsystem = subsystem;
    addRequirements(subsystem);
  }

  @Override
  public void initialize()
  {
    liftSubsystem.setMotorPower(-.5);
  }

  @Override
  public void end(boolean interrupted)
  {
    liftSubsystem.setMotorPower(0);
  }
}
