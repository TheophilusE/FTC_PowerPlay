package org.firstinspires.ftc.teamcode.statemachine.triggers;

import org.firstinspires.ftc.teamcode.statemachine.delegates.Action;
import org.firstinspires.ftc.teamcode.statemachine.delegates.FuncBoolean;

public class InternalTriggerBehaviour<S, T> extends TriggerBehaviour<S, T>
{
  private final Action action;

  public InternalTriggerBehaviour(T trigger, FuncBoolean guard, Action action)
  {
    super(trigger, guard);
    this.action = action;
  }

  @Override
  public void performAction(Object[] args)
  {
    action.doIt();
  }

  @Override
  public boolean isInternal()
  {
    return true;
  }

  @Override
  public S transitionsTo(S source, Object[] args)
  {
    return source;
  }
}
