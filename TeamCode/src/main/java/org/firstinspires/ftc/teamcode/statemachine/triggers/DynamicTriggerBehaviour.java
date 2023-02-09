package org.firstinspires.ftc.teamcode.statemachine.triggers;

import org.firstinspires.ftc.teamcode.statemachine.delegates.Action1;
import org.firstinspires.ftc.teamcode.statemachine.delegates.Func2;
import org.firstinspires.ftc.teamcode.statemachine.delegates.FuncBoolean;

public class DynamicTriggerBehaviour<S, T> extends TriggerBehaviour<S, T>
{

  private final Func2<Object[], S> destination;
  private final Action1<Object[]>  action;

  public DynamicTriggerBehaviour(T trigger, Func2<Object[], S> destination, FuncBoolean guard, Action1<Object[]> action)
  {
    super(trigger, guard);
    assert destination != null : "destination is null";
    this.destination = destination;
    this.action      = action;
  }

  @Override
  public void performAction(Object[] args)
  {
    action.doIt(args);
  }

  @Override
  public S transitionsTo(S source, Object[] args)
  {
    return destination.call(args);
  }
}
