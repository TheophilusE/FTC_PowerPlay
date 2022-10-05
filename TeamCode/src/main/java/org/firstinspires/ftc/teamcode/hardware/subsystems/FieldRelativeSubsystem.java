package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.control.util.Extensions;

public class FieldRelativeSubsystem extends SubsystemBase
{
  private final BNO055IMU imu;
  double offset;

  public FieldRelativeSubsystem(final HardwareMap hardwareMap)
  {
    offset = Extensions.HEADING;

    imu = hardwareMap.get(BNO055IMU.class, "imu");
    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
    parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
    imu.initialize(parameters);
  }

  public void updateHeading()
  {
    offset += getAngle();
  }

  public double getAngle()
  {
    double angle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle;
    angle = AngleUnit.normalizeRadians(angle - offset);
    return angle;
  }

  public double getAngle(double angleOffset)
  {
    double angle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle;
    angle = AngleUnit.normalizeRadians(angle - angleOffset);
    return angle;
  }
}
