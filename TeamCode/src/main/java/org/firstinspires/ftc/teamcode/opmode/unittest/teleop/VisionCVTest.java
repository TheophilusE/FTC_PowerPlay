package org.firstinspires.ftc.teamcode.opmode.unittest.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.control.derived.AutonomousUtils;
import org.firstinspires.ftc.teamcode.hardware.subsystems.VisionCVSubsystem;
import org.firstinspires.ftc.teamcode.opmode.OpModeBase;

@TeleOp(name = "VisionCVTest", group = "TeleOpUnitTest")
public class VisionCVTest extends OpModeBase
{
  @Override
  public void initialize()
  {
    super.initialize();

    AutonomousUtils.Initialize();
  }

  @Override
  public void registerSubsystems()
  {
    super.registerSubsystems();
    addSubsystem(new VisionCVSubsystem(hardwareMap, telemetry));
  }

  @Override
  public void update()
  {
    // Get and update Vision CV
    {
      VisionCVSubsystem visionCVSubsystem = getComponent(VisionCVSubsystem.class);
      if (visionCVSubsystem != null)
      {
      }
    }
  }
}
