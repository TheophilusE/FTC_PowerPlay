package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.sun.tools.javac.util.Pair;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.control.algorithm.PIDControl;

import java.util.ArrayList;

@Config
public class LiftSubsystem extends SubsystemBase
{
  public enum LiftLevel
  {
    ZERO_LEVEL,
    ONE_LEVEL,
    TWO_LEVEL,
    THREE_LEVEL
  }

  // Motors
  private final DcMotorSimple  liftMotor;
  private final DistanceSensor distanceSensor;

  // Control System
  PIDControl liftPIDControl;

  // Control Data
  private double currentPosition;
  private double targetPosition;

  private ArrayList<Pair<LiftLevel, Integer>> liftLevelPairs = new ArrayList<Pair<LiftLevel, Integer>>(4);

  public static double  LIFT_COEFFICIENT = 1.0;
  public static boolean enableTracking   = false;

  // Construct
  public LiftSubsystem(final HardwareMap hardwareMap, final String motorName, final String sensorName)
  {
    // Retrieve accessors hardware
    liftMotor      = hardwareMap.get(DcMotorSimple.class, motorName);
    distanceSensor = hardwareMap.get(DistanceSensor.class, sensorName);

    // Initialize controller
    liftPIDControl = new PIDControl(0.5, 0.3, 0.2);

    // Set initial control data
    currentPosition = distanceSensor.getDistance(DistanceUnit.INCH);
    targetPosition  = currentPosition;

    // Set up parameters for lifting platform motor
    liftPIDControl.setSetpoint(0);
    liftPIDControl.setOutputRange(0, 1.0);
    liftPIDControl.setInputRange(0, 36);
    liftPIDControl.enable();

    // Fill level pairs
    liftLevelPairs.add(new Pair<LiftLevel, Integer>(LiftLevel.ZERO_LEVEL, 3));
    liftLevelPairs.add(new Pair<LiftLevel, Integer>(LiftLevel.ONE_LEVEL, 7));
    liftLevelPairs.add(new Pair<LiftLevel, Integer>(LiftLevel.TWO_LEVEL, 12));
    liftLevelPairs.add(new Pair<LiftLevel, Integer>(LiftLevel.THREE_LEVEL, 22));
  }

  @Override
  public void periodic()
  {
    // Early exit if tracking is disabled or set to false
    if (!enableTracking)
    {
      return;
    }

    if (liftPIDControl.getSetpoint() != targetPosition)
    {
      liftPIDControl.setSetpoint(targetPosition);
    }

    currentPosition = distanceSensor.getDistance(DistanceUnit.INCH);

    // Set the current position that will be used to calculate the error
    // and generate our final output power.
    double motorPower = liftPIDControl.performPID(currentPosition);
    liftMotor.setPower(motorPower);
  }

  public double getTargetPosition()
  {
    return targetPosition;
  }

  public void setTargetPosition(LiftLevel liftLevel)
  {
    for (int i = 0; i < liftLevelPairs.size(); ++i)
    {
      if (liftLevelPairs.get(i).fst == liftLevel)
      {
        targetPosition = liftLevelPairs.get(i).snd.intValue();
      }
    }
  }

  public double getCurrentDistanceToFloor()
  {
    currentPosition = distanceSensor.getDistance(DistanceUnit.INCH); // Update current position
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

  public DcMotorSimple getLiftMotor()
  {
    return liftMotor;
  }
}
