package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.opmode.Defines;
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

  @Override
  public void periodic()
  {
    visionCV.update(telemetry);
  }

  public Defines.ParkTargetSignal getTargetSignal()
  {
    return visionCV.getPipeline().getTargetSignal();
  }

  public VisionCV getVisionCV()
  {
    return visionCV;
  }
}
