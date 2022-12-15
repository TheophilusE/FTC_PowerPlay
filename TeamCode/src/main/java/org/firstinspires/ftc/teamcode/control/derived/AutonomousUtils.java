package org.firstinspires.ftc.teamcode.control.derived;

import static org.firstinspires.ftc.teamcode.opmode.Defines.BLUE_ALLIANCE;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.opmode.Defines;

/*
 * Various Autonomous Utilities.
 */
public final class AutonomousUtils
{
  public static void InitializeHeading()
  {
    Defines.START_POSE = new Pose2d(6, BLUE_ALLIANCE ? 63 : -63,
                                    Math.toRadians(BLUE_ALLIANCE ? -90 : 90));
  }
}
