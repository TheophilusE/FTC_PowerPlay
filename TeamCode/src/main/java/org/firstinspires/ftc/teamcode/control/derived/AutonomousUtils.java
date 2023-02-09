package org.firstinspires.ftc.teamcode.control.derived;

import static org.firstinspires.ftc.teamcode.opmode.Defines.BLUE_ALLIANCE;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.opmode.Defines;

/*
 * Various Autonomous Utilities.
 */
public final class AutonomousUtils
{
  public static void Initialize()
  {
    if (!Defines.FSM_STATE_OVERRIDE)
    {
      Defines.autonomousFSMState = Defines.AutonomousFSMState.IDLE_STATE;
    }

    Defines.START_POSE = new Pose2d(6, BLUE_ALLIANCE ? 63 : -63,
                                    Math.toRadians(BLUE_ALLIANCE ? -90 : 90));
  }
}
