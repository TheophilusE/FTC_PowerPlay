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
  }

  @Override
  public void periodic()
  {
    // This method will be called once per scheduler run
  }
}
