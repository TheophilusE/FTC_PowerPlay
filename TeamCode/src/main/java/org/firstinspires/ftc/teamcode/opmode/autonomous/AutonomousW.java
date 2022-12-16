package org.firstinspires.ftc.teamcode.opmode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@TeleOp(name = "Autonomous")
public class AutonomousW extends LinearOpMode{

  private DcMotor leftRear;
  private DcMotor rightRear;
  private DcMotor rightFront;
  private DcMotor leftFront;

  private OpenCvWebcam webcam;
  private ColorDetermination.CupDeterminationPipeline pipeline;
  private ColorDetermination.CupDeterminationPipeline.CupColor snapshotAnalysis = ColorDetermination.CupDeterminationPipeline.CupColor.RED;

  @Override
  public void runOpMode() {



    leftFront = hardwareMap.get(DcMotor.class, "leftFront");
    rightFront = hardwareMap.get(DcMotor.class, "rightFront");
    rightRear = hardwareMap.get(DcMotor.class, "rightRear");
    leftRear = hardwareMap.get(DcMotor.class, "leftRear");

    leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
    leftRear.setDirection(DcMotorSimple.Direction.REVERSE);

    int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
    webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam"), cameraMonitorViewId);
    pipeline = new ColorDetermination.CupDeterminationPipeline();
    webcam.setPipeline(pipeline);

    webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
      @Override
      public void onOpened() {
        webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
      }

      @Override
      public void onError(int errorCode) {
      }
    });
    while(!isStarted() && !isStopRequested()) {
      telemetry.addData("Anaylsis:", pipeline.getAnalysis());

      telemetry.addData("Y:",pipeline.Y);
      telemetry.addData("Cr:",pipeline.Cr);
      telemetry.addData("Cb:",pipeline.Cb);
      telemetry.update();
    }

    if (pipeline.getAnalysis() == ColorDetermination.CupDeterminationPipeline.CupColor.BLUE) {
      leftFront.setPower(1);
      rightFront.setPower(1);
      leftRear.setPower(1);
      rightRear.setPower(1);
      sleep(1220);
      leftFront.setPower(1);
      rightFront.setPower(-1);
      leftRear.setPower(1);
      rightRear.setPower(-1);
      sleep(1150);
      leftFront.setPower(-1);
      rightFront.setPower(-1);
      leftRear.setPower(-1);
      rightRear.setPower(-1);
      sleep(800);
      leftFront.setPower(0);
      rightFront.setPower(0);
      leftRear.setPower(0);
      rightRear.setPower(0);
    }
    else if (pipeline.getAnalysis() == ColorDetermination.CupDeterminationPipeline.CupColor.GREEN) {
      leftFront.setPower(1);
      rightFront.setPower(1);
      leftRear.setPower(1);
      rightRear.setPower(1);
      sleep(1220);
      leftFront.setPower(0);
      rightFront.setPower(0);
      leftRear.setPower(0);
      rightRear.setPower(0);
    }
    else {
      leftFront.setPower(1);
      rightFront.setPower(1);
      leftRear.setPower(1);
      rightRear.setPower(1);
      sleep(1300);
      leftFront.setPower(-1);
      rightFront.setPower(1);
      leftRear.setPower(-1);
      rightRear.setPower(1);
      sleep(1020);
      leftFront.setPower(-1);
      rightFront.setPower(-1);
      leftRear.setPower(-1);
      rightRear.setPower(-1);
      sleep(700);
      leftFront.setPower(0);
      rightFront.setPower(0);
      leftRear.setPower(0);
      rightRear.setPower(0);

    }
  }
}