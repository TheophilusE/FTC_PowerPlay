package org.firstinspires.ftc.teamcode.opmode.unittest.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.hardware.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.opmode.OpModeBase;

@Config
@TeleOp(name = "Lift Test", group = "TeleOpUnitTest")
public class LiftTest extends OpModeBase
{
  public static AutoState state            = AutoState.ONE_LEVEL;
  public        int       currentLiftLevel = 0;

  CRServo motor = null;

  @Override
  public void initialize()
  {
    super.initialize();
    motor = hardwareMap.get(CRServo.class, "liftMotor");
  }

  @Override
  public void update()
  {
    motor.setPower(1);
    driveEngine.setZeroPower();
  }

  public enum AutoState
  {
    ZERO_LEVEL,
    ONE_LEVEL,
    TWO_LEVEL,
    THREE_LEVEL
  }
}
