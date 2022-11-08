package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

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

  private ElapsedTime elapsedTime       = null;
  private double      targetElapsedTime = 0.0;

  public VisionCVSubsystem(final HardwareMap hardwareMap, final Telemetry telemetry)
  {
    visionCV       = new VisionCV(hardwareMap);
    this.telemetry = telemetry;
    elapsedTime    = new ElapsedTime();
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

  public void shutdown()
  {
    visionCV.shutDown();
  }
}
