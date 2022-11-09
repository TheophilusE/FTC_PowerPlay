package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.control.algorithm.PIDControl;

@Config
public class LiftSubsystem extends SubsystemBase
{
  // Motors
  private final DcMotorEx liftMotor;

  // Control System
  PIDControl liftPIDControl;

  // Control Data
  private double currentPosition;
  private double targetPosition;

  public static double LIFT_COEFFICIENT = 1.0;

  // Construct
  public LiftSubsystem(final HardwareMap hardwareMap, final String name)
  {
    liftMotor = hardwareMap.get(DcMotorEx.class, name);
    liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    // Initialize controller
    liftPIDControl = new PIDControl(0.5, 0.3, 0.2);

    // Set initial control data
    currentPosition = liftMotor.getCurrentPosition();
    targetPosition  = currentPosition;

    // Set up parameters for lifting platform motor
    liftPIDControl.setSetpoint(0);
    liftPIDControl.setOutputRange(0, 1.0);
    liftPIDControl.setInputRange(0, 10000);
    liftPIDControl.enable();
  }

  @Override
  public void periodic()
  {
    if (liftPIDControl.getSetpoint() != targetPosition)
    {
      liftPIDControl.setSetpoint(targetPosition);
    }

    currentPosition = liftMotor.getCurrentPosition();

    // Set the current position that will be used to calculate the error
    // and generate our final output power.
    double motorPower = liftPIDControl.performPID(currentPosition);
    liftMotor.setPower(motorPower);
  }

  public double getTargetPosition()
  {
    return targetPosition;
  }

  public void setTargetPosition(double position)
  {
    targetPosition = position;
  }

  public double getCurrentPosition()
  {
    currentPosition = liftMotor.getCurrentPosition(); // Update current position
    return currentPosition;
  }

  public double getMotorPower()
  {
    return liftMotor.getPower();
  }

  public void setMotorPower(double power)
  {
    liftMotor.setPower(power);
  }

  public DcMotorEx getLiftMotor()
  {
    return liftMotor;
  }
}
