package org.firstinspires.ftc.teamcode.opmode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.control.derived.DriveEngine;
import org.firstinspires.ftc.teamcode.hardware.commands.BulkCacheCommand;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public abstract class OpModeBase extends CommandOpMode
{
  // Tracked Variables
  private ElapsedTime elapsedTime         = null;
  private double      executionsPerSecond = 1.0 / 60.0;
  private double      robotAngleOffset    = 0;
  private boolean     previousState       = false;

  // Engine Core
  protected DriveEngine driveEngine = null;

  // Accessors

  // Sensors

  // Containers
  /**
   * The list of components this entity is composed of.
   */
  private final List<SubsystemBase> subsystems = new ArrayList<>();

  /**
   * Provides fast access to components based on its type.
   */
  private final Map<Class<?>, SubsystemBase> cache = new HashMap<>();

  @Override
  public void initialize()
  {
    // Clear any previous states
    reset();

    // Clear the bulk cache command every iteration
    schedule(new BulkCacheCommand(hardwareMap));

    driveEngine = new DriveEngine(hardwareMap);

    registerAccessors();

    registerSubsystems();

    // Instantiate a new elapsed time object.
    if (elapsedTime != null)
    {
      elapsedTime = new ElapsedTime();
    }
  }

  @Override
  public void run()
  {
    super.run();

    update();

    // Temporarily halt updates until total execution time is reached
    while (elapsedTime.milliseconds() < executionsPerSecond)
    {
    }

    // Prepare counter for next execution.
    elapsedTime.reset();
  }

  /*
   * Provides an update method that must be implemented by derived classes.
   */
  public abstract void update();

  @Override
  public void reset()
  {
    super.reset();

    // Clear Elapsed Time
    if (elapsedTime != null)
    {
      elapsedTime.reset();
    }
  }

  public void setExecutionsPerSecond(double executionsPerSecond)
  {
    this.executionsPerSecond = 1.0 / executionsPerSecond;
  }

  /* Register motors and sensors */
  public void registerAccessors()
  {
  }

  /* Register subsystems */
  public void registerSubsystems()
  {
  }

  /**
   * Returns {@code true} if this entity has a component of the specified
   * type. A type can either be an interface of a class.
   *
   * <pre>
   *
   * if (opmode.hasComponent(Foo.class))
   * {
   *     System.out.println(&quot;Opmode has component of type Foo&quot;);
   * }
   * </pre>
   *
   * @param classRef the class or interface the requested component must implement
   * @return {@code true} if the requested component is part of this opmode
   */
  public boolean hasSubsystem(Class<?> classRef)
  {
    if (cache.containsKey(classRef))
    {
      return true;
    }

    for (SubsystemBase subsystem : subsystems)
    {
      if (classRef.isInstance(subsystem))
      {
        return true;
      }
    }
    return false;
  }

  /**
   * Retrieves the subsystem of this entity of the specified type. A type can
   * either be an interface of a class.
   *
   * <p>
   * If more than one subsystem implements the specified type, the first
   * component that matches the requirements will be returned.
   * </p>
   *
   * <p>
   * This method returns throws an {@code IllegalArgumentException} in case no
   * subsystem can be found that matches the requested type. This is a
   * deliberate design decision to avoid repetitive checks against
   * {@code null} references. In almost all cases one can assume that the
   * requested subsystem must be part of the used entity. If this fails, this
   * is most likely an error condition that should be indicated as fast as
   * possible (<i>fail-fast</i>). In cases where it is not certain that an
   * entity has contains a certain type of component the {@link #hasSubsystem(Class)}
   * can be used test this.
   * </p>
   *
   * <p>
   * <strong>Example</strong>
   * </p>
   *
   * <pre>
   * public interface CollisionHandler {
   *     public void handleCollision();
   * }
   *
   * public class Foo extends Component implements CollisionHandler {
   *
   *     public void handleCollision() {
   *         //...
   *     }
   *
   *     public void doSomething() {
   *         //...
   *     }
   *
   *     //...
   * }
   *
   * Entity bar = getEntityFromSomeWhere();
   * bar.addComponent(new Foo());
   * //..
   *
   * // Invoke collision handling without knowing about Foo
   * bar.getComponent(CollisionHandler.class).handleCollision();
   * //..
   *
   * // Retrieve subsystem Foo
   * Foo foo = opmode.getComponent(Foo.class);
   * foo.doSomething();
   * </pre>
   *
   * @param <T>      type parameter used to avoid casts, irrelevant when calling
   *                 this method
   * @param classRef the class or interface the requested component must implement
   * @return the subsystem that implements the specified interface of class
   * @throws IllegalArgumentException if no subsystem with the specified type could be found
   */
  public <T> T getComponent(Class<T> classRef) throws IllegalArgumentException
  {
    SubsystemBase cached = cache.get(classRef);

    if (cached != null)
    {
      return classRef.cast(cached);
    }

    for (SubsystemBase system : subsystems)
    {
      if (classRef.isInstance(system))
      {
        cache.put(classRef, system);
        return classRef.cast(system);
      }
    }
    throw new IllegalArgumentException("Subsystem not found "
                                       + classRef.getName());
  }

  /**
   * Retrieves a list of all subsystems of the specified type.
   *
   * <p>
   * <strong>Note: </strong> A new list object will be created each time this
   * method is called. Do not use this method too often.
   * </p>
   *
   * @param <T>      type parameter used to avoid casts, irrelevant when calling
   *                 this method
   * @param classRef the class or interface the requested components must implement
   * @return a list with components of the specified type (may be empty)
   */
  public <T> List<T> getAllComponents(Class<T> classRef)
  {
    ArrayList<T> result = new ArrayList<T>();

    for (SubsystemBase system : subsystems)
    {
      if (classRef.isInstance(system))
      {
        result.add(classRef.cast(system));
      }
    }

    return result;
  }

  /**
   * Adds the specified subsystem to this opmode. Subsystem cannot be added if
   * this subsystem already exists in this opmode.
   *
   * @param system the component to be added
   */
  public void addSubsystem(SubsystemBase system)
  {
    for (SubsystemBase s : getAllComponents(system.getClass()))
    {
      if (s == system)
      {
        throw new IllegalArgumentException(
            "Subsystem already added to OpMode");
      }
    }

    subsystems.add(system);
  }
}