package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.vision.VisionCV;

/*
 * A subsystem that is used to aid the abstraction of Computer Vision.
 */
public class VisionCVSubsystem extends SubsystemBase
{
  private final VisionCV  visionCV;
  private final Telemetry telemetry;

  public VisionCVSubsystem(final HardwareMap hardwareMap, final Telemetry telemetry)
  {
    visionCV       = new VisionCV(hardwareMap);
    this.telemetry = telemetry;
  }

  public int runCameraDetection(float timeInSeconds, boolean shutdown)
  {
    ElapsedTime time = new ElapsedTime();

    while (time.seconds() < timeInSeconds)
    {
      visionCV.update(telemetry);
    }

    if (shutdown)
    {
      return visionCV.shutDown();
    }
    return 0;
  }

  @Override
  public void periodic()
  {
    visionCV.update(telemetry);
  }

  /*
  public Defines.HubLevel getCurrentHubLevel()
  {
    return visionCV.getPipeline().getHubLevel();
  }
   */

  public void shutdown()
  {
    visionCV.shutDown();
  }
}
