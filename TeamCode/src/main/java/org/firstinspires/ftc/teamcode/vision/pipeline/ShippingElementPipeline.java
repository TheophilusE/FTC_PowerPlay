package org.firstinspires.ftc.teamcode.vision.pipeline;

import com.sun.tools.javac.util.Pair;

import org.firstinspires.ftc.teamcode.math.FastMath;
import org.firstinspires.ftc.teamcode.math.Vector3d;
import org.firstinspires.ftc.teamcode.opmode.Defines;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.function.Function;

/*
 * Pipeline for OpenCV Color Detection.
 */
public class ShippingElementPipeline extends OpenCvPipeline
{
  // Readup the following
  // - https://en.wikipedia.org/wiki/YCbCr
  // - https://dsp.stackexchange.com/questions/2687/why-do-we-use-the-hsv-colour-space-so-often-in-vision-and-image-processing
  // - https://learnmediatech.com/what-is-ycbcr/

  public static double centerMarkerPositionX = 0.5;
  public static double centerMarkerPositionY = 0.5;

  public static int centerMarkerPositionWidth  = 40;
  public static int centerMarkerPositionHeight = 40;

  public static double   minThresholdValue = 200; // Smaller values equal more precision
  private final Mat      yCrCbMat          = new Mat();
  public        Vector3d yCrCbFinalColor   = new Vector3d();
  public        Vector3d rgbFinalColor     = new Vector3d();

  // volatile because it's accessed by the opmode thread with no sync
  private volatile Defines.ParkTargetSignal                            targetSignal          = Defines.ParkTargetSignal.SIGNAL_NONE;
  private final    ArrayList<Pair<Defines.ParkTargetSignal, Vector3d>> signalColorPairs      = new ArrayList<Pair<Defines.ParkTargetSignal, Vector3d>>(3);
  private final    ArrayList<Pair<Defines.ParkTargetSignal, Vector3d>> signalColorPairsYCrCB = new ArrayList<Pair<Defines.ParkTargetSignal, Vector3d>>(signalColorPairs.size());

  public ShippingElementPipeline()
  {
    // Populate signal pairs
    signalColorPairs.add(new Pair<Defines.ParkTargetSignal, Vector3d>(Defines.ParkTargetSignal.SIGNAL_ONE, new Vector3d(41.0, 118.0, 187.0)));
    signalColorPairs.add(new Pair<Defines.ParkTargetSignal, Vector3d>(Defines.ParkTargetSignal.SIGNAL_TWO, new Vector3d(51.0, 184.0, 100.0)));
    signalColorPairs.add(new Pair<Defines.ParkTargetSignal, Vector3d>(Defines.ParkTargetSignal.SIGNAL_THREE, new Vector3d(253.0, 89.0, 86.0)));

    // Compute YCrCb Values
    for (int i = 0; i < signalColorPairs.size(); ++i)
    {
      int[] result = FastMath.convertRGB2YCRCB((int) signalColorPairs.get(i).snd.x, (int) signalColorPairs.get(i).snd.y, (int) signalColorPairs.get(i).snd.z);
      signalColorPairsYCrCB.add(new Pair<Defines.ParkTargetSignal, Vector3d>(signalColorPairs.get(i).fst, new Vector3d(result[0], result[1], result[2])));
    }
  }

  @Override
  public Mat processFrame(Mat input)
  {
    // Convert to the YCrCb color space from RGB
    Imgproc.cvtColor(input, yCrCbMat, Imgproc.COLOR_RGB2YCrCb);

    // Create the sample region (s)
    Rect centerSampleRect = new Rect(
        (int) (centerMarkerPositionX * input.width()),
        (int) (centerMarkerPositionY * input.height()),
        centerMarkerPositionWidth,
        centerMarkerPositionHeight
    );

    // Submat our sample regions
    Mat centerSampleRegion    = yCrCbMat.submat(centerSampleRect);
    Mat rgbCenterSampleRegion = input.submat(centerSampleRect);

    Scalar centerRegionMean    = Core.mean(centerSampleRegion);
    Scalar rgbCenterRegionMean = Core.mean(rgbCenterSampleRegion);

    // Detect marker type and deduce marker color.
    boolean centerMarkerDetected = false;
    {
      Defines.ParkTargetSignal bestDetectedSignal = Defines.ParkTargetSignal.SIGNAL_NONE;
      yCrCbFinalColor = new Vector3d(centerRegionMean.val[0], centerRegionMean.val[1], centerRegionMean.val[2]);
      rgbFinalColor   = new Vector3d(rgbCenterRegionMean.val[0], rgbCenterRegionMean.val[1], rgbCenterRegionMean.val[2]);
      double detectThreshold = minThresholdValue;

      for (int i = 0; i < signalColorPairsYCrCB.size(); ++i)
      {
        // Get the distance from the current final color to our expected output color.
        // We could use the Square distance instead to be more efficient but this is done
        // For the sake of simplicity. If there are any performance degradations, use the
        // Distance Squared value to avoid the square root.
        double distance = rgbFinalColor.distance(signalColorPairs.get(i).snd);

        // Set our new threshold to the current distance, as we are trying to get the closest color.
        if (distance < detectThreshold)
        {
          bestDetectedSignal = signalColorPairs.get(i).fst;
          detectThreshold    = distance;
        }
      }

      targetSignal = bestDetectedSignal;

      if (bestDetectedSignal != Defines.ParkTargetSignal.SIGNAL_NONE)
      {
        centerMarkerDetected = true;
      }
    }

    if (centerMarkerDetected)
    {
      // TODO: Add extra logic if marker detected
    }

    // Now that we have all the data we need here, we can start putting things on the viewport for debugging

    // Draw the sample regions

    // Center sample region
    switch (targetSignal)
    {
      case SIGNAL_NONE:
        Imgproc.rectangle(input, centerSampleRect, new Scalar(255.0, 255.0, 255.0), 2);
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
    rgbCenterSampleRegion.release();

    return input;
  }

  public Defines.ParkTargetSignal getTargetSignal()
  {
    return targetSignal;
  }
}