package org.firstinspires.ftc.teamcode.vision;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.opmode.Defines;
import org.firstinspires.ftc.teamcode.vision.pipeline.ShippingElementPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

public class VisionCV
{
  private final OpenCvWebcam            webcam;
  private final ShippingElementPipeline pipeline;

  public VisionCV(HardwareMap hardwareMap)
  {
    // OpenCV webcam
    int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
    webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
    //OpenCV Pipeline

    // Set the Pipeline
    pipeline = new ShippingElementPipeline();
    webcam.setPipeline(pipeline);

    /*
     * Open the connection to the camera device. New in v1.4.0 is the ability
     * to open the camera asynchronously, and this is now the recommended way
     * to do it. The benefits of opening async include faster init time, and
     * better behavior when pressing stop during init (i.e. less of a chance
     * of tripping the stuck watchdog)
     *
     * If you really want to open synchronously, the old method is still available.
     */
    webcam.setMillisecondsPermissionTimeout(2500); // Timeout for obtaining permission is configurable. Set before opening.
    webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
    {
      @Override
      public void onOpened()
      {
        webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
      }

      @Override
      public void onError(int errorCode)
      {
        // Handle error here
      }
    });

    // Stream to FTC Dashboard. Set the ENABLE_CAMERA_STREAM to false to disable.
    if (Defines.ENABLE_CAMERA_STREAM)
    {
      FtcDashboard.getInstance().startCameraStream(webcam, Defines.STREAM_MAX_FPS);
    }
  }

  public void update(Telemetry telemetry)
  {
    telemetry.addData("Frame Count", webcam.getFrameCount());
    telemetry.addData("FPS", String.format("%.2f", webcam.getFps()));
    telemetry.addData("Total frame time ms", webcam.getTotalFrameTimeMs());
    telemetry.addData("Pipeline time ms", webcam.getPipelineTimeMs());
    telemetry.addData("Overhead time ms", webcam.getOverheadTimeMs());
    telemetry.addData("Theoretical max FPS", webcam.getCurrentPipelineMaxFps());

    // telemetry.addData("Current Hub Level", pipeline.getHubLevel());

    //telemetry.addData("left cb", pipeline.leftRegionCb);
    //telemetry.addData("center cb", pipeline.centerRegionCb);
  }

  public double inValues(double value, double min, double max)
  {
    if (value < min)
    {
      value = min;
    }
    if (value > max)
    {
      value = max;
    }
    return value;
  }

  public int shutDown()
  {
    webcam.stopStreaming();
    webcam.closeCameraDevice();

    return 0;
  }

  public OpenCvCamera getWebcam()
  {
    return webcam;
  }

  public ShippingElementPipeline getPipeline()
  {
    return pipeline;
  }
}