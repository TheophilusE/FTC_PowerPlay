package org.firstinspires.ftc.teamcode.opmode.unittest.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.opmode.OpModeBase;


@TeleOp(name = "XDriveTest", group = "TeleOpUnitTest")
public class HolonomicDriveTest extends OpModeBase
{
  @Override
  public void update()
  {
    // [-1: Left, 1: Right]
    // This is reversed -> [-1: Up, 1: Down]

    double y  = -gamepad1.left_stick_y; // Remember, this is reversed!
    double x  = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
    double rx = gamepad1.right_stick_x;

    // Denominator is the largest motor power (absolute value) or 1
    // This ensures all the powers maintain the same ratio, but only when
    // at least one is out of the range [-1, 1]
    double denominator     = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
    double leftFrontPower  = (y + x + rx) / denominator;
    double leftRearPower   = (y - x + rx) / denominator;
    double rightFrontPower = (y - x - rx) / denominator;
    double rightRearPower  = (y + x - rx) / denominator;

    driveEngine.setMotorPowers(leftFrontPower, leftRearPower, rightRearPower, rightFrontPower);
  }
}
