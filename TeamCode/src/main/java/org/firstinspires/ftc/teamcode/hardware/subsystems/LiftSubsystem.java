package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.control.algorithm.PIDControl;

// import static org.firstinspires.ftc.teamcode.control.util.Defines.LIFT_MOTOR_NAME;

public class LiftSubsystem extends SubsystemBase
{
  // Motors
  private final DcMotorEx liftMotor;

  // Control System
  PIDControl liftPIDControl;

  // Control Data
  private double currentPosition;
  private double targetPosition;
  private double liftCoefficient;

  // Construct
  public LiftSubsystem(final HardwareMap hardwareMap, final String name)
  {
    liftMotor = hardwareMap.get(DcMotorEx.class, name);
    liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    liftCoefficient = 1.0;

    // Initialize controller
    liftPIDControl = new PIDControl(0.5, 0.3, 0.2);

    // Set initial control data
    currentPosition = liftMotor.getCurrentPosition();
    targetPosition  = currentPosition;

    // Set up parameters for lifting platform motor
    liftPIDControl.setSetpoint(0);
    liftPIDControl.setOutputRange(0, liftCoefficient);
    liftPIDControl.setInputRange(-90, 90);
    liftPIDControl.enable();
  }

  public double getTargetPositionError()
  {
    return targetPosition - currentPosition;
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

  public double getLiftCoefficient()
  {
    return liftCoefficient;
  }

  public void setLiftCoefficient(double coefficient)
  {
    liftCoefficient = coefficient;
  }

  public double getMotorPower()
  {
    return liftMotor.getPower();
  }

  public void setMotorPower(double power)
  {
    liftMotor.setPower(power);
  }
}
