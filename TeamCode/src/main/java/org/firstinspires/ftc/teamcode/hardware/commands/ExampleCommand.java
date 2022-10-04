package org.firstinspires.ftc.teamcode.hardware.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.hardware.subsystems.ExampleSubsystem;

/*
 * An example command that uses an example subsystem.
 */
public class ExampleCommand extends CommandBase
{
  // The subsystem that this command runs on.
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ExampleSubsystem subsystem;

  /*
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ExampleCommand(ExampleSubsystem subsystem)
  {
    this.subsystem = subsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }
}
