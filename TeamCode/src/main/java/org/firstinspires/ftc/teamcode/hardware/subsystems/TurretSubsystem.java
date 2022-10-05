package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.control.util.Encoder;

public class TurretSubsystem extends FieldRelativeSubsystem
{
  private final DcMotorEx turretControlMotor;
  private       Encoder   turretController;

  public TurretSubsystem(final HardwareMap hardwareMap, final String name)
  {
    super(hardwareMap);

    turretControlMotor = hardwareMap.get(DcMotorEx.class, name);
    turretController   = new Encoder(turretControlMotor);
  }

  @Override
  public void periodic()
  {
    // Link - https://www.dynapar.com/knowledge/applications/angle_encoders/

    double heading = getAngle();
  }
}
