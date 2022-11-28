package org.firstinspires.ftc.teamcode.opmode.unittest.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.opmode.Defines;
import org.firstinspires.ftc.teamcode.opmode.OpModeBase;

@TeleOp(name = "MecanumDriveTest", group = "TeleOpUnitTest")
public class MecanumTrigDriveTest extends OpModeBase
{
  @Override
  public void update()
  {
    // [-1: Left, 1: Right]
    // This is reversed -> [-1: Up, 1: Down]

    double length   = Math.hypot(gamepad1.left_stick_x, -gamepad1.left_stick_y);
    double angle    = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) - (Math.PI * 0.25);
    double rotation = gamepad1.right_stick_x;

    double leftFrontPower  = length * Math.cos(angle);
    double leftRearPower   = length * Math.sin(angle);
    double rightRearPower  = length * Math.cos(angle);
    double rightFrontPower = length * Math.sin(angle);

    // Restrict all power values such that the length is less than or equal to 1
    // as defined by the Unit Circle.
    leftFrontPower /= length;
    leftRearPower /= length;
    rightRearPower /= length;
    rightFrontPower /= length;

    // Apply Rotation
    leftFrontPower += rotation;
    leftRearPower += rotation;
    rightRearPower -= rotation;
    rightFrontPower -= rotation;

    // Ensure that the length of the vector is the 1
    double rotLength = Math.hypot(leftFrontPower, rightFrontPower);
    leftFrontPower /= rotLength;
    leftRearPower /= rotLength;
    rightRearPower /= rotLength;
    rightFrontPower /= rotLength;

    telemetry.addData("> ", " Length:   (%.2f)", length);
    telemetry.addData("> ", " Angle:    (%.2f)", angle);
    telemetry.addData("> ", " Rotation: (%.2f)", rotation);

    driveEngine.setMotorPowers(leftFrontPower * Defines.DRIVE_COEFFICIENT,
                               leftRearPower * Defines.DRIVE_COEFFICIENT,
                               rightRearPower * Defines.DRIVE_COEFFICIENT,
                               rightFrontPower * Defines.DRIVE_COEFFICIENT
                              );
  }
}
