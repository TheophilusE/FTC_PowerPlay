package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.sun.tools.javac.util.Pair;

import java.util.ArrayList;

public class ClawSubsystem extends SubsystemBase
{
  /*
   * Enumeration of all supported servo positions.
   */
  public enum ClawPosition
  {
    OPEN,
    CLOSE,

    ENUM_COUNT
  }

  /*
   * Defines a claw index, either left or right.
   */
  public enum ClawIndex
  {
    LEFT,
    RIGHT,

    ENUM_COUNT
  }

  // Servos
  private final Servo servoLeft;
  private final Servo servoRight;

  // Data
  private ClawPosition currentClawPosition = ClawPosition.ENUM_COUNT;

  private ArrayList<Pair<ClawPosition, double[]>> clawPositionPairs = new ArrayList<Pair<ClawPosition, double[]>>(2);

  // Construct
  public ClawSubsystem(final HardwareMap hardwareMap, String leftClawName, String rightClawName)
  {
    // Retrieve hardware interfacing object
    servoLeft  = hardwareMap.get(Servo.class, leftClawName);
    servoRight = hardwareMap.get(Servo.class, rightClawName);

    // TODO: Fill claw positions
    clawPositionPairs.add(new Pair<ClawPosition, double[]>(ClawPosition.CLOSE, new double[]{0.0, 0.3}));
    clawPositionPairs.add(new Pair<ClawPosition, double[]>(ClawPosition.OPEN, new double[]{0.35, 0.0}));
  }

  @Override
  public void periodic()
  {
    // This method will be called once per scheduler run
    {
      double currentPositionValue = getClawPositionValue(currentClawPosition, ClawIndex.LEFT);
      if (servoLeft.getPosition() != currentPositionValue)
      {
        servoLeft.setPosition(currentPositionValue);
      }
    }

    {
      double currentPositionValue = getClawPositionValue(currentClawPosition, ClawIndex.RIGHT);
      if (servoRight.getPosition() != currentPositionValue)
      {
        servoRight.setPosition(currentPositionValue);
      }
    }
  }

  public ClawPosition getCurrentClawPosition()
  {
    return currentClawPosition;
  }

  public void setClawPosition(ClawPosition clawPosition)
  {
    currentClawPosition = clawPosition;
  }

  /*
   * Retrieve the value of a claw position.
   * Returns NaN if the a value cannot be determined in the case of ENUM_COUNT.
   */
  public double getClawPositionValue(ClawPosition clawPosition, ClawIndex clawIndex)
  {
    assert (clawPosition != ClawPosition.ENUM_COUNT) : "Claw position must be less than ENUM_COUNT";
    assert (clawIndex.ordinal() < ClawIndex.ENUM_COUNT.ordinal()) : "Claw position must be less than ENUM_COUNT";

    for (int i = 0; i < clawPositionPairs.size(); ++i)
    {
      if (clawPositionPairs.get(i).fst == clawPosition)
      {
        return clawPositionPairs.get(i).snd[clawIndex.ordinal()];
      }
    }
    return Double.NaN;
  }

  public Servo getLeftServo()
  {
    return servoLeft;
  }

  public Servo getRightServo()
  {
    return servoRight;
  }
}
