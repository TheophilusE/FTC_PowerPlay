package org.firstinspires.ftc.teamcode.opmode.unittest.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.opmode.OpModeBase;

@TeleOp(name = "MechanumDriveTest", group = "TeleOpUnitTest")
public class MechanumTrigDriveTest extends OpModeBase
{
  @Override
  public void update()
  {
    // [-1: Left, 1: Right]
    // This is reversed -> [-1: Up, 1: Down]

    double length     = Math.hypot(gamepad1.left_stick_x, -gamepad1.left_stick_y);
    double robotAngle = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
    double rightX     = gamepad1.right_stick_x;

    final double leftFrontPower  = length * Math.cos(robotAngle) + rightX;
    final double rightFrontPower = length * Math.sin(robotAngle) - rightX;
    final double leftRearPower   = length * Math.sin(robotAngle) + rightX;
    final double rightRearPower  = length * Math.cos(robotAngle) - rightX;

    driveEngine.setMotorPowers(leftFrontPower, leftRearPower, rightRearPower, rightFrontPower);
  }
}
