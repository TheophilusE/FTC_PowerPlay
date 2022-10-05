package org.firstinspires.ftc.teamcode.hardware.commands;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.subsystems.CarouselSubsystem;

/*
 * A simple command that spins the carousel wheel with the
 * {@link CarouselSubsystem}.
 */
public class CarouselCommand extends ParallelCommandGroup
{
  // The subsystem the command runs on.
  private final CarouselSubsystem carouselSubsystem;
  // Time in seconds to spin carousel wheel.
  double      time;
  // Keep track of elapsed time.
  ElapsedTime elapsedTime = null;
  // Is command complete flag
  boolean     isComplete  = false;

  // Construct
  public CarouselCommand(CarouselSubsystem subsystem)
  {
    carouselSubsystem = subsystem;
    time              = 0;
    addRequirements(carouselSubsystem);
  }

  public CarouselCommand(CarouselSubsystem subsystem, double timeInSeconds)
  {
    carouselSubsystem = subsystem;
    time              = timeInSeconds;
    addRequirements(carouselSubsystem);
  }

  @Override
  public void initialize()
  {
    if (time <= 0)
    {
      carouselSubsystem.spinPlatformMotor(0.5);
    } else
    {
      elapsedTime = new ElapsedTime();
      carouselSubsystem.spinPlatformMotorWithTime(0.5, time);
    }
  }

  @Override
  public void execute()
  {
    if (!(time <= 0) && elapsedTime.seconds() > time)
    {
      carouselSubsystem.stopPlatformMotor();
      isComplete = true;
    }
  }

  @Override
  public boolean isFinished()
  {
    return carouselSubsystem.isClear() && isComplete;
  }

  @Override
  public void end(boolean interrupted)
  {
    carouselSubsystem.stopPlatformMotor();
  }
}
