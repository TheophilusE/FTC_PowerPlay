package org.firstinspires.ftc.teamcode.hardware.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.hardware.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.hardware.subsystems.ExampleSubsystem;

public class ClawCommand extends CommandBase
{
  // The subsystem that this command runs on.

  private final ClawSubsystem subsystem;

  /*
   * Creates a new Claw Command.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ClawCommand(ClawSubsystem subsystem, double servoLeftPosition, double servoRightPosition)
  {
    this.subsystem = subsystem;

    subsystem.setServoPositions(servoLeftPosition, servoRightPosition);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  /*
   * Returns a status update whether the command has finished.
   */
  @Override
  public boolean isFinished()
  {
    return true;
  }

  /*
   * This is run before the command ends execution, the command may have been forced to end.
   *
   * @param interrupted specifies if the command was interrupted; forced to quit.
   */
  @Override
  public void end(boolean interrupted)
  {
  }
}
