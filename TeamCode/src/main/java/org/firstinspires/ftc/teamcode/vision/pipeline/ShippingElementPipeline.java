package org.firstinspires.ftc.teamcode.vision.pipeline;

import android.graphics.Bitmap;

import com.acmerobotics.dashboard.config.Config;

import org.opencv.android.Utils;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.function.Function;

@Config
public class ShippingElementPipeline extends OpenCvPipeline
{
  public static double leftMarkerPositionX = 0.25;
  public static double leftMarkerPositionY = 0.5;

  public static int leftMarkerPositionWidth  = 20;
  public static int leftMarkerPositionHeight = 20;

  public static double centerMarkerPositionX = 0.5;
  public static double centerMarkerPositionY = 0.5;

  public static int centerMarkerPositionWidth  = 20;
  public static int centerMarkerPositionHeight = 20;

  public static   int    thresholdValue = 150;
  final           Object sync           = new Object();
  // volatile because reasons
  public volatile double leftRegionCb   = 0;
  public volatile double centerRegionCb = 0;

  // We are going to assume the marker is solid yellow here.
  // Ideally because we can control the color, a solid white or black would be good, or something like that
  // volatile because it's accessed by the opmode thread with no sync
  // private volatile Defines.HubLevel hubLevel       = Defines.HubLevel.BOTTOM;
  private final Mat            yCrCbMat       = new Mat();
  private final Mat            cBMat          = new Mat();
  private       MatSavingState matSavingState = MatSavingState.NONE_REQUESTED;
  private final Mat            matToSave      = new Mat();

  /*
  public Defines.HubLevel getHubLevel()
  {
    return hubLevel;
  }
   */

  @Override
  public Mat processFrame(Mat input)
  {
    // Convert to the YCrCb color space from RGB
    Imgproc.cvtColor(input, yCrCbMat, Imgproc.COLOR_RGB2YCrCb);

    // Extract the Cb (blue-difference) channel
    Core.extractChannel(yCrCbMat, cBMat, 2);

    // Make the sample regions
    Rect leftSampleRect = new Rect(
        (int) (leftMarkerPositionX * input.width()),
        (int) (leftMarkerPositionY * input.height()),
        leftMarkerPositionWidth,
        leftMarkerPositionHeight
    );

    Rect centerSampleRect = new Rect(
        (int) (centerMarkerPositionX * input.width()),
        (int) (centerMarkerPositionY * input.height()),
        centerMarkerPositionWidth,
        centerMarkerPositionHeight
    );

    // submat our sample regions
    Mat leftSampleRegion   = cBMat.submat(leftSampleRect);
    Mat centerSampleRegion = cBMat.submat(centerSampleRect);

    // Find the average color value of each the regions
    Scalar leftRegionMean   = Core.mean(leftSampleRegion);
    Scalar centerRegionMean = Core.mean(centerSampleRegion);

    // Read the first channel, the only channel in this case since we extracted it before.
    leftRegionCb   = leftRegionMean.val[0];
    centerRegionCb = centerRegionMean.val[0];

    // Test the amount against our threshold to see if there is a marker there.
    boolean leftMarkerDetected   = (leftRegionCb > thresholdValue); // see if it is blue
    boolean centerMarkerDetected = (centerRegionCb > thresholdValue); // see if it is blue

    //if both are detected just choose whichever one is more blue
    if (leftMarkerDetected && centerMarkerDetected)
    {
      if (leftRegionMean.val[0] >= centerRegionMean.val[0])
      { //include the rare equals case in this because /s
        centerMarkerDetected = false;
      } else
      {
        leftMarkerDetected = false;
      }
    }

    /*

    // Set the appropriate levels respectively
    if (leftMarkerDetected)
    {
      hubLevel = Defines.HubLevel.BOTTOM;
    } else if (centerMarkerDetected)
    {
      hubLevel = Defines.HubLevel.MIDDLE;
    } else
    {
      hubLevel = Defines.HubLevel.TOP;
    }


    // Now that we have all the data we need here, we can start putting things on the viewport for debugging
    // Draw the sample regions

    // Left Region, bottom level
    switch (hubLevel)
    {
      case TOP:
      case MIDDLE:
        Imgproc.rectangle(input, leftSampleRect, new Scalar(190.0, 40.0, 70.0), 2);
        break;
      case BOTTOM:
        Imgproc.rectangle(input, leftSampleRect, new Scalar(20.0, 220.0, 70.0), 2);
        break;
    }

    // Center region, middle level
    switch (hubLevel)
    {
      case BOTTOM:
      case TOP:
        Imgproc.rectangle(input, centerSampleRect, new Scalar(190.0, 40.0, 70.0), 2);
        break;
      case MIDDLE:
        Imgproc.rectangle(input, centerSampleRect, new Scalar(20.0, 220.0, 70.0), 2);
        break;
    }

    // Write some text on the viewport
    Imgproc.putText(input,
                    ((Function<Defines.HubLevel, String>) hublevel ->
                    {
                      switch (hubLevel)
                      {
                        case BOTTOM:
                          return "Bottom Level";
                        case MIDDLE:
                          return "Middle Level";
                        case TOP:
                          return "Top Level";
                        default:
                          return "";
                      }
                    }).apply(hubLevel),
                    new Point(0.25 * input.width(), 0.2 * input.height()),
                    Imgproc.FONT_HERSHEY_TRIPLEX,
                    0.8,
                    new Scalar(255.0, 50.0, 0.0),
                    2
                   );

     */

    // Make sure to release the mats we created in the pipeline, otherwise we get mem leaks
    leftSampleRegion.release();
    centerSampleRegion.release();

    // Me playing with mat to bitmap conversion, don't worry about this
    synchronized (sync)
    {
      if (matSavingState == MatSavingState.MAT_REQUESTED)
      {
        input.copyTo(matToSave);
        matSavingState = MatSavingState.MAT_READY_FOR_CONVERSION;
      }
    }

    return input;
  }

  public void storeNextMat()
  {
    synchronized (sync)
    {
      matSavingState = MatSavingState.MAT_REQUESTED;
    }
  }

  public Bitmap getCurrentBitmap()
  {
    synchronized (sync)
    {
      if (matSavingState == MatSavingState.MAT_READY_FOR_CONVERSION)
      {
        final Bitmap bitmap = Bitmap.createBitmap(matToSave.cols(), matToSave.rows(), Bitmap.Config.RGB_565);
        Utils.matToBitmap(matToSave, bitmap);
        matSavingState = MatSavingState.NONE_REQUESTED;
        return bitmap;
      } else
      {
        return null;
      }
    }
  }


  // All this is me messing around with converting a mat to a bitmap, it's not relevant to the pipeline
  private enum MatSavingState
  {
    NONE_REQUESTED,
    MAT_REQUESTED,
    MAT_READY_FOR_CONVERSION
  }
}