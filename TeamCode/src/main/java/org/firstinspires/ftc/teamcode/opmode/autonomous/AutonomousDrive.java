package org.firstinspires.ftc.teamcode.opmode.autonomous;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.opmode.Defines;
import org.firstinspires.ftc.teamcode.opmode.OpModeBase;

@Config
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
