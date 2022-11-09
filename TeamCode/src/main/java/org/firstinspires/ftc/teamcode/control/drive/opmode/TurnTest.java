package org.firstinspires.ftc.teamcode.control.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.control.derived.DriveEngine;

/*
 * This is a simple routine to test turning capabilities.
 */
@Config
@Autonomous(name = "Turn Test", group = "DriveTuner")
public class TurnTest extends LinearOpMode
{
  public static double ANGLE = 90; // deg

  @Override
  public void runOpMode() throws InterruptedException
  {
    DriveEngine drive = new DriveEngine(hardwareMap);

    waitForStart();

    if (isStopRequested()) return;

    drive.turn(Math.toRadians(ANGLE));
  }
}
