package org.firstinspires.ftc.teamcode.opmode.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmode.Defines;
import org.firstinspires.ftc.teamcode.opmode.OpModeBase;

/*
 * This class houses the main autonomous program that enables full functionality.
 */

@Config
@Autonomous(name = "Autonomous Drive", group = "Autonomous")
public class AutonomousDrive extends OpModeBase
{
  @Override
  public void initialize()
  {
    super.initialize();

    if (!Defines.FSM_STATE_OVERRIDE)
    {
      Defines.autonomousFSM = Defines.AutonomousFSM.IDLE;
    }
  }

  @Override
  public void update()
  {
    // Update Finite State Machine
    updateFSM();
  }

  public void updateFSM()
  {
    switch (Defines.autonomousFSM)
    {
      default:
      {
        driveEngine.setZeroPower();
      }
      break;
    }
  }
}
