package org.firstinspires.ftc.teamcode.opmode.unittest.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.hardware.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.hardware.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.opmode.OpModeBase;

@Config
@TeleOp(name = "Lift Test", group = "TeleOpUnitTest")
public class LiftTest extends OpModeBase
{


  @Override
  public void initialize()
  {
    super.initilize();
  }

  @Override
  public void update()
  {
  }

  @Override
  public void registerSubsystems()
  {
    super.registerSubsystems();

    // Register Lift Subsystem
    {
      telemetry.addLine("> Register Lift Subsystem...");

      LiftSubsystem liftSubsystem = new LiftSubsystem(hardwareMap, "liftMotor", "colorDistanceSensor");
      liftSubsystem.enableTracking = false;
      addSubsystem(liftSubsystem);

      telemetry.update();
    }

    // Register Claw Subsystem
    {
      telemetry.addLine("> Register Claw Subsystem...");

      addSubsystem(new ClawSubsystem(hardwareMap));

      telemetry.update();
    }
  }
}


