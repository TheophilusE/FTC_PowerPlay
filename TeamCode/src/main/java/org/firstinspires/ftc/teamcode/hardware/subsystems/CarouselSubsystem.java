package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * A rotating mechanism that spins the carousel wheel until a duck
 * is delivered into the playing field.
 */
public class CarouselSubsystem extends SubsystemBase
{
  // Motor reference
  private final DcMotorEx platformMotor;
  // Time to spin the wheel
  double time;
  // Is spinning platform motor.
  private       boolean isBusy;
  // Forward or reverse direction.
  private final boolean isForwardDirection;

  // Construct
  public CarouselSubsystem(final HardwareMap hardwareMap, final String name)
  {
    platformMotor = hardwareMap.get(DcMotorEx.class, name);

    platformMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    isBusy             = false;
    isForwardDirection = false;
    time               = 0;
  }

  public CarouselSubsystem(final HardwareMap hardwareMap, final String name, boolean forward)
  {
    platformMotor = hardwareMap.get(DcMotorEx.class, name);
    platformMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    isBusy             = false;
    isForwardDirection = forward;
    time               = 0;
  }

  public CarouselSubsystem(final HardwareMap hardwareMap, String name, double timeInSeconds, boolean forward)
  {
    platformMotor = hardwareMap.get(DcMotorEx.class, name);
    platformMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    isBusy             = false;
    isForwardDirection = forward;
    time               = timeInSeconds;
  }

  /* Spin carousel wheel */
  public void spinPlatformMotor(double speed)
  {
    platformMotor.setPower(isForwardDirection ? speed : -speed);
  }

  /* Stop carousel wheel */
  public void stopPlatformMotor()
  {
    platformMotor.setPower(0);
  }

  /* Spin carousel with time */
  public void spinPlatformMotorWithTime(double speed, double timeInSeconds)
  {
    ElapsedTime elapsedTime = new ElapsedTime();
    isBusy = true;

    while (elapsedTime.seconds() <= timeInSeconds)
    {
      platformMotor.setPower(isForwardDirection ? speed : -speed);
    }

    platformMotor.setPower(0);
    isBusy = false;
  }

  /* Return true if platform motor is not busy */
  public boolean isClear()
  {
    return !isBusy;
  }

  /* Get the current power level of the platform motor */
  public double getCurrentPower()
  {
    return platformMotor.getPower();
  }

  /* Set the current power level of the platform motor */
  public void setCurrentPower(double power)
  {
    platformMotor.setPower(power);
  }
}
