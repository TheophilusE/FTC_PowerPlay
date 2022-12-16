/*
 * Copyright (c) 2020 OpenFTC Team
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
package org.firstinspires.ftc.teamcode.opmode.autonomous;

import android.graphics.ColorSpace;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

/*
 * This sample demonstrates a basic (but battle-tested and essentially
 * 100% accurate) method of detecting the cup when lined up with
 * the sample regions over the first 3 stones.
 */
@TeleOp(name = "ZeAuto")
public class ColorDetermination extends LinearOpMode {
  OpenCvInternalCamera webCam;
  CupDeterminationPipeline pipeline;
  private DcMotor backRight  = null;
  private DcMotor frontRight = null;
  private DcMotor frontLeft  = null;
  private DcMotor backLeft   = null;
  private DcMotor liftMotor = null;
  double liftPower;
  @Override
  public void runOpMode() {
    /*
     * NOTE: Many comments have been omitted from this sample for the
     * sake of conciseness. If you're just starting out with EasyOpenCv,
     * you should take a look at {@link InternalCamera1Example} or its
     * webcam counterpart, {@link WebcamExample} first.
     */
    frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
    backLeft = hardwareMap.get(DcMotor.class, "backLeft");
    frontRight = hardwareMap.get(DcMotor.class, "frontRight");
    backRight = hardwareMap.get(DcMotor.class, "backRight");
    liftMotor =  hardwareMap.get(DcMotor.class, "armMotor" );

    int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
    webCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
    pipeline = new CupDeterminationPipeline();
    webCam.setPipeline(pipeline);

    // We set the viewport policy to optimized view so the preview doesn't appear 90 deg
    // out when the RC activity is in portrait. We do our actual image processing assuming
    // landscape orientation, though.
    webCam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);

    webCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
      @Override
      public void onOpened() {
        webCam.startStreaming(640, 480, OpenCvCameraRotation.SIDEWAYS_LEFT);
      }

      @Override
      public void onError(int errorCode) {
        /*
         * This will be called if the camera could not be opened
         */
      }
    });

    waitForStart();

    while (opModeIsActive()) {
      telemetry.addData("Anaylsis:", pipeline.getAnalysis());

      telemetry.addData("Y:",pipeline.Y);
      telemetry.addData("Cr:",pipeline.Cr);
      telemetry.addData("Cb:",pipeline.Cb);
      telemetry.update();


    }
  }

  public static class CupDeterminationPipeline extends OpenCvPipeline {
    /*
     * Some color constants
     */
    static final Scalar BLUE = new Scalar(0, 0, 255);
    static final Scalar GREEN = new Scalar(0, 255, 0);
    static final Scalar RED = new Scalar(255, 0, 0);
    /*
     * The core values which define the location and size of the sample regions
     */
    static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(310, 230);
    static final int REGION_WIDTH = 20;
    static final int REGION_HEIGHT = 20;
    /*
     * Points which actually define the sample region rectangles, derived from above values
     *
     * Example of how points A and B work to define a rectangle
     *
     *   ------------------------------------
     *   | (0,0) Point A                    |
     *   |                                  |
     *   |                                  |
     *   |                                  |
     *   |                                  |
     *   |                                  |
     *   |                                  |
     *   |                  Point B (70,50) |
     *   ------------------------------------
     *
     */
    Point region1_pointA = new Point(
        REGION1_TOPLEFT_ANCHOR_POINT.x,
        REGION1_TOPLEFT_ANCHOR_POINT.y);
    Point region1_pointB = new Point(
        REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
        REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
    /*
     * Working variables
     */
    Mat region1;
    Mat YCrCb = new Mat();
    Mat RGB = new Mat();
    Mat RGBregion1;
    int avg1= 0;
    int Y = 0;
    public int Cr = 0;
    public int Cb = 0;

    // Volatile since accessed by OpMode thread w/o synchronization
    private volatile CupColor position = CupColor.RED;

    /*
     * This function takes the RGB frame, converts to YCrCb,
     * and extracts the Cb channel to the 'Cb' variable
     */
    void inputToCb(Mat input) {
      Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
    }

    @Override
    public void init(Mat firstFrame) {
      /*
       * We need to call this in order to make sure the 'Cb'
       * object is initialized, so that the submats we make
       * will still be linked to it on subsequent frames. (If
       * the object were to only be initialized in processFrame,
       * then the submats would become delinked because the backing
       * buffer would be re-allocated the first time a real frame
       * was crunched)
       */
      inputToCb(firstFrame);

      /*
       * Submats are a persistent reference to a region of the parent
       * buffer. Any changes to the child affect the parent, and the
       * reverse also holds true.
       */
      region1 = YCrCb.submat(new Rect(region1_pointA, region1_pointB));
      RGB = YCrCb.clone();
      Imgproc.cvtColor(RGB,RGB,Imgproc.COLOR_YCrCb2RGB);
      RGBregion1 = RGB.submat(new Rect(region1_pointA, region1_pointB));
    }

    @Override
    public Mat processFrame(Mat input) {
      /*
       * Overview of what we're doing:
       *
       * We first convert to YCrCb color space, from RGB color space.
       * Why do we do this? Well, in the RGB color space, chroma and
       * luma are intertwined. In YCrCb, chroma and luma are separated.
       * YCrCb is a 3-channel color space, just like RGB. YCrCb's 3 channels
       * are Y, the luma channel (which essentially just a B&W image), the
       * Cr channel, which records the difference from red, and the Cb channel,
       * which records the difference from blue. Because chroma and luma are
       * not related in YCrCb, vision code written to look for certain values
       * in the Cr/Cb channels will not be severely affected by differing
       * light intensity, since that difference would most likely just be
       * reflected in the Y channel.

       */
      inputToCb(input);

      /*
       * Compute the average pixel value of each submat region. We're
       * taking the average of a single channel buffer, so the value
       * we need is at index 0. We could have also taken the average
       * pixel value of the 3-channel image, and referenced the value
       * at index 2 here.
       */

      for(int i = 0; i<REGION_HEIGHT; i++){
        for (int j = 0; j<REGION_WIDTH; j++){
          Y += (int) region1.get(i,j)[0];
          Cr += (int) region1.get(i,j)[1];
          Cb += (int) region1.get(i,j)[2];
        }
      }

      Y = Y/(REGION_HEIGHT*REGION_WIDTH);
      Cr = Cr/(REGION_HEIGHT*REGION_WIDTH);
      Cb = Cb/(REGION_HEIGHT*REGION_WIDTH);


      /*
       * Draw a rectangle showing sample region 1 on the screen.
       * Simply a visual aid. Serves no functional purpose.
       */
      Imgproc.rectangle(
          input, // Buffer to draw on
          region1_pointA, // First point which defines the rectangle
          region1_pointB, // Second point which defines the rectangle
          BLUE, // The color the rectangle is drawn in
          2); // Thickness of the rectangle lines

      /*
       * Now that we found the AVG, we actually need to go and
       * figure out which sample region that value was from
       */
      if ((27  < Y && Y<125) && (175 < Cr && Cr < 250) && (95 < Cb && Cb < 135)) // Was it from region 1?
      {
        position = CupColor.RED; // Record our analysis

        /*
         * Draw a solid rectangle on top of the chosen region.
         * Simply a visual aid. Serves no functional purpose.
         */
        Imgproc.rectangle(
            input, // Buffer to draw on
            region1_pointA, // First point which defines the rectangle
            region1_pointB, // Second point which defines the rectangle
            RED, // The color the rectangle is drawn in
            -1); // Negative thickness means solid fill
      } else if ((45<Y && Y<160)&&(55 < Cr && Cr < 105)&&(85 < Cb && Cb < 135)) // Was it from region 2?
      {
        position = CupColor.GREEN; // Record our analysis

        /*
         * Draw a solid rectangle on top of the chosen region.
         * Simply a visual aid. Serves no functional purpose.
         */
        Imgproc.rectangle(
            input, // Buffer to draw on
            region1_pointA, // First point which defines the rectangle
            region1_pointB, // Second point which defines the rectangle
            GREEN, // The color the rectangle is drawn in
            -1); // Negative thickness means solid fill
      } else if ((20 <Y && Y<115)&&(75 < Cr && Cr < 125)&&(130 < Cb && Cb < 210)) // Was it from region 3?
      {
        position = CupColor.BLUE; // Record our analysis

        /*
         * Draw a solid rectangle on top of the chosen region.
         * Simply a visual aid. Serves no functional purpose.
         */
        Imgproc.rectangle(
            input, // Buffer to draw on
            region1_pointA, // First point which defines the rectangle
            region1_pointB, // Second point which defines the rectangle
            BLUE, // The color the rectangle is drawn in
            -1); // Negative thickness means solid fill
      }

      /*
       * Render the 'input' buffer to the viewport. But note this is not
       * simply rendering the raw camera feed, because we called functions
       * to add some annotations to this buffer earlier up.
       */
      return input;
    }

    /*
     * Call this from the OpMode thread to obtain the latest analysis
     */
    public CupColor getAnalysis() {
      return position;
    }

    /*
     * An enum to define the cup position
     */
    public enum CupColor {
      RED,
      GREEN,
      BLUE
    }
  }
}