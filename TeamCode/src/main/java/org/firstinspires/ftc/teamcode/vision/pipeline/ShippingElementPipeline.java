package org.firstinspires.ftc.teamcode.vision.pipeline;

import com.sun.tools.javac.util.Pair;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.teamcode.math.Vector3f;
import org.firstinspires.ftc.teamcode.math.Vector4f;
import org.firstinspires.ftc.teamcode.opmode.Defines;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.function.Function;

/*
 * Pipeline for OpenCV Color Detection.
 */
public class ShippingElementPipeline extends OpenCvPipeline
{
  public static double centerMarkerPositionX = 0.5;
  public static double centerMarkerPositionY = 0.5;

  public static int centerMarkerPositionWidth  = 20;
  public static int centerMarkerPositionHeight = 20;

  public static int thresholdValue = 120;

  // volatile because it's accessed by the opmode thread with no sync
  private volatile Defines.ParkTargetSignal                            targetSignal     = Defines.ParkTargetSignal.SIGNAL_NONE;
  private          ArrayList<Pair<Defines.ParkTargetSignal, Vector4f>> signalColorPairs = new ArrayList<Pair<Defines.ParkTargetSignal, Vector4f>>(3);

  // Target color during comparison
  private Vector4f targetColor = new Vector4f(1.0f, 1.0f, 1.0f, 1.0f);

  private final Mat yCrCbMat = new Mat();
  private final Mat cBMat    = new Mat();

  public ShippingElementPipeline()
  {
    // Populate signal pairs
    new Pair<Defines.ParkTargetSignal, Vector4f>(Defines.ParkTargetSignal.SIGNAL_ONE, new Vector4f(0.0f, 0.0f, 0.0f, 1.0f));
    signalColorPairs.add(new Pair<Defines.ParkTargetSignal, Vector4f>(Defines.ParkTargetSignal.SIGNAL_ONE, new Vector4f(0.0f, 0.0f, 0.0f, 1.0f)));
    signalColorPairs.add(new Pair<Defines.ParkTargetSignal, Vector4f>(Defines.ParkTargetSignal.SIGNAL_TWO, new Vector4f(0.0f, 0.0f, 0.0f, 1.0f)));
    signalColorPairs.add(new Pair<Defines.ParkTargetSignal, Vector4f>(Defines.ParkTargetSignal.SIGNAL_THREE, new Vector4f(0.0f, 0.0f, 0.0f, 1.0f)));
  }

  @Override
  public Mat processFrame(Mat input)
  {
    // Convert to the YCrCb color space from RGB
    Imgproc.cvtColor(input, yCrCbMat, Imgproc.COLOR_RGB2YCrCb);

    // Extract the Cb (blue-difference) channel
    Core.extractChannel(yCrCbMat, cBMat, 2);

    // Create the sample region (s)
    Rect centerSampleRect = new Rect(
        (int) (centerMarkerPositionX * input.width()),
        (int) (centerMarkerPositionY * input.height()),
        centerMarkerPositionWidth,
        centerMarkerPositionHeight
    );

    // Submat our sample regions
    Mat centerSampleRegion = cBMat.submat(centerSampleRect);

    Scalar centerRegionMean = Core.mean(centerSampleRegion);

    // Detect marker type here
    boolean centerMarkerDetected = false;

    // if both are detected
    if (centerMarkerDetected)
    {
      // Deduce color

      // Set the appropriate levels respectively
    }

    // Now that we have all the data we need here, we can start putting things on the viewport for debugging

    // Draw the sample regions

    // Center sample region
    switch (targetSignal)
    {
      case SIGNAL_NONE:
        Imgproc.rectangle(input, centerSampleRect, new Scalar(255.0f, 255.0, 255.0), 2);
        break;
      case SIGNAL_ONE:
        Imgproc.rectangle(input, centerSampleRect, new Scalar(signalColorPairs.get(0).snd.x, signalColorPairs.get(0).snd.y, signalColorPairs.get(0).snd.z), 2);
        break;
      case SIGNAL_TWO:
        Imgproc.rectangle(input, centerSampleRect, new Scalar(signalColorPairs.get(1).snd.x, signalColorPairs.get(1).snd.y, signalColorPairs.get(1).snd.z), 2);
        break;
      case SIGNAL_THREE:
        Imgproc.rectangle(input, centerSampleRect, new Scalar(signalColorPairs.get(2).snd.x, signalColorPairs.get(2).snd.y, signalColorPairs.get(2).snd.z), 2);
        break;
    }

    // Write info text on the viewport.
    Imgproc.putText(input,
                    ((Function<Defines.ParkTargetSignal, String>) hublevel ->
                    {
                      switch (targetSignal)
                      {
                        case SIGNAL_NONE:
                          return "Alliance Park";
                        case SIGNAL_ONE:
                          return "Left Park";
                        case SIGNAL_TWO:
                          return "Middle Park";
                        case SIGNAL_THREE:
                          return "Right Park";
                        default:
                          return "";
                      }
                    }).apply(targetSignal),
                    new Point(0.5 * input.width(), 0.2 * input.height()),
                    Imgproc.FONT_HERSHEY_COMPLEX,
                    1.0,
                    new Scalar(255.0, 255.0, 255.0)
                   );

    // Release to avoid memory leaks
    centerSampleRegion.release();

    return input;
  }

  public Defines.ParkTargetSignal getTargetSignal()
  {
    return targetSignal;
  }
}