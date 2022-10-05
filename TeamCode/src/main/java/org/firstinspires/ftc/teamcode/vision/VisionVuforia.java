/*
 * Copyright (c) 2020-2022 Theophilus Eriata.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.vision;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

public class VisionVuforia
{
  private static final String           VUFORIA_KEY = "AeadzuH/////AAABmdXJEThQTkAoulqgbT87Ql2GRM51eA6BnJ8K4kZVIlgRqiAPWB1I/MXJIsry32qWadcSRRRInBTRHIxVRhtsIrOhnM+1bXFTopE4n6/3/EZ4HdLfOB3pnouUXpBKIx3z8ubYMcHJ1gXVoPO8h6fZyURNhWLvTufKP0FvuDMTvnTb2pnxod5vMP1i1DRYrYKMZcwQznRS4tlM5a+yCdGUKRdit5JGx5ClctXIEEyGCyisWSvWglilVOeQi5a4pgjp3KmHJacFiFRTJubphUyB5rVtoKV0yozFK1xztnSrSEHXWcOiM2WUL4swQKnfPl3qzZRcoc83GF+JSQ5eEMKbvrxd62FtBgkPsDVjOCP+pq/Q";
  protected            VuforiaLocalizer vuforia     = null;
  protected            OpenCvCamera     openCvCamera;

  public static boolean ENABLE_CAMERA_STREAM = false;

  public VisionVuforia(HardwareMap hardwareMap)
  {
    int   cameraMonitorViewId  = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
    int[] viewportContainerIds = OpenCvCameraFactory.getInstance().splitLayoutForMultipleViewports(cameraMonitorViewId, 2, OpenCvCameraFactory.ViewportSplitMethod.VERTICALLY);

    // Setup Vuforia
    VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(viewportContainerIds[0]);
    parameters.vuforiaLicenseKey = VUFORIA_KEY;
    parameters.cameraDirection   = VuforiaLocalizer.CameraDirection.BACK;
    // Uncomment this line below to use a webcam
    //parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
    vuforia = ClassFactory.getInstance().createVuforia(parameters);

    // Create a Vuforia passthrough "virtual camera"
    openCvCamera = OpenCvCameraFactory.getInstance().createVuforiaPassthrough(vuforia, parameters, viewportContainerIds[1]);

    openCvCamera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
    {
      @Override
      public void onOpened()
      {
        // Using GPU acceleration can be particularly helpful when using Vuforia passthrough
        // mode, because Vuforia often chooses high resolutions (such as 720p) which can be
        // very CPU-taxing to rotate in software. GPU acceleration has been observed to cause
        // issues on some devices, though, so if you experience issues you may wish to disable it.
        openCvCamera.setViewportRenderer(OpenCvCamera.ViewportRenderer.GPU_ACCELERATED);
        openCvCamera.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);
        openCvCamera.setPipeline(new VisionVuforia.ColorBoxDrawingPipeline(new Scalar(255, 0, 0, 255)));

        // We don't get to choose resolution, unfortunately. The width and height parameters
        // are entirely ignored when using Vuforia passthrough mode. However, they are left
        // in the method signature to provide interface compatibility with the other types
        // of cameras.
        openCvCamera.startStreaming(0, 0, OpenCvCameraRotation.SIDEWAYS_LEFT);
      }

      @Override
      public void onError(int errorCode)
      {
        /*
         * This will be called if the camera could not be opened
         */
      }
    });

    // Stream to FTC Dashboard. Set the ENABLE_CAMERA_STREAM to false to disable.
    if (ENABLE_CAMERA_STREAM)
    {
      FtcDashboard.getInstance().startCameraStream(openCvCamera, 10);
    }
  }

  public OpenCvCamera getOpenCvCamera()
  {
    return openCvCamera;
  }

  class ColorBoxDrawingPipeline extends OpenCvPipeline
  {
    Scalar color;

    ColorBoxDrawingPipeline(Scalar color)
    {
      this.color = color;
    }

    @Override
    public Mat processFrame(Mat input)
    {
      Imgproc.rectangle(
          input,
          new Point(
              input.cols() / 4,
              input.rows() / 4),
          new Point(
              input.cols() * (3f / 4f),
              input.rows() * (3f / 4f)),
          color, 4);

      return input;
    }
  }
}
