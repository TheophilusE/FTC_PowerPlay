package org.firstinspires.ftc.teamcode.opmode.unittest.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.control.derived.AutonomousUtils;
import org.firstinspires.ftc.teamcode.hardware.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.hardware.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.opmode.Defines;
import org.firstinspires.ftc.teamcode.opmode.OpModeBase;

@Config
@TeleOp(name = "Lift Test", group = "TeleOpUnitTest")
public class LiftTest extends OpModeBase
{
  @Override
  public void initialize()
  {
    super.initialize();

    AutonomousUtils.InitializeHeading();
  }

  @Override
  public void update()
  {
  }

  @Override
  public void registerSubsystems()
  {
    super.registerSubsystems();

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
}


