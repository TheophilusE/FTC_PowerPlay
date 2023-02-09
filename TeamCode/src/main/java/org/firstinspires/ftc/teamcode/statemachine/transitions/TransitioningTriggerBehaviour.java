package org.firstinspires.ftc.teamcode.statemachine.transitions;

import org.firstinspires.ftc.teamcode.statemachine.delegates.Action;
import org.firstinspires.ftc.teamcode.statemachine.delegates.FuncBoolean;
import org.firstinspires.ftc.teamcode.statemachine.triggers.TriggerBehaviour;

public class TransitioningTriggerBehaviour<S, T> extends TriggerBehaviour<S, T>
{

  private final S      destination;
  private final Action action;

  public TransitioningTriggerBehaviour(T trigger, S destination, FuncBoolean guard, Action action)
  {
    super(trigger, guard);
    this.destination = destination;
    this.action      = action;
  }

  @Override
  public void performAction(Object[] args)
  {
    action.doIt();
  }

  @Override
  public S transitionsTo(S source, Object[] args)
  {
    return destination;
  }
}
