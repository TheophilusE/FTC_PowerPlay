package org.firstinspires.ftc.teamcode.statemachine.delegates;

/**
 * Tracing delegate allows one to investigate state machine working at runtime.
 *
 * @param <S> State type
 * @param <T> Trigger type
 * @see org.firstinspires.ftc.teamcode.statemachine.StateMachine#setTrace(Trace)
 */
public interface Trace<S, T>
{

  /**
   * This callback is called each time a trigger is fired, before evaluation,
   * allowing to trace all events sent to the sate machine
   *
   * @param trigger Trigger sent to the state machine
   */
  void trigger(T trigger);

  /**
   * This callback is called each time a transition is performed, after trigger evaluation.
   *
   * @param trigger     Trigger sent to the state machine
   * @param source      Source state
   * @param destination Destination state
   */
  void transition(T trigger, S source, S destination);
}
