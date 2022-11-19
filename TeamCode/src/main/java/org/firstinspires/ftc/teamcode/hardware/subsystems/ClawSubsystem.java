package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ClawSubsystem extends SubsystemBase
{
  private final Servo servoLeft;
  private final Servo servoRight;

  // Construct
  public ClawSubsystem(final HardwareMap hardwareMap)
  {
    servoLeft = hardwareMap.get(Servo.class, "leftClaw");
    servoRight = hardwareMap.get(Servo.class, "rightClaw");

    servoRight.setDirection(Servo.Direction.REVERSE);

    // Set servo initial position
    servoRight.setPosition(1);
    servoLeft.setPosition(1);
  }

  public ClawSubsystem(final HardwareMap hardwareMap, double leftPosition, double rightPosition)
  {
    servoLeft = hardwareMap.get(Servo.class, "leftClaw");
    servoRight = hardwareMap.get(Servo.class, "rightClaw");

    servoRight.setDirection(Servo.Direction.REVERSE);

    // Set servo initial position
    servoRight.setPosition(rightPosition);
    servoLeft.setPosition(leftPosition);
  }

  public void setServoPositions(double servoLeftPosition, double servoRightPosition)
  {
    servoLeft.setPosition(servoLeftPosition);
    servoRight.setPosition(servoRightPosition);
  }

  public void setServoLeftPosition(double position)
  {
    servoLeft.setPosition(position);
  }

  public void setServoRightPosition(double position)
  {
    servoRight.setPosition(position);
  }

  @Override
  public void periodic()
  {
    // This method will be called once per scheduler run
  }

  public Servo getServoLeft()
  {
    return servoLeft;
  }

  public Servo getServoRight()
  {
    return servoRight;
  }
}
