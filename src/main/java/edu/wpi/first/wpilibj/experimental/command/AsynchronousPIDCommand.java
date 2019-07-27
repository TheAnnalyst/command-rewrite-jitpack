package edu.wpi.first.wpilibj.experimental.command;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.PIDControllerRunner;

import java.util.Set;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

import static java.util.Objects.requireNonNull;

/**
 * A command that controls an output with a {@link PIDController}.  Runs forever by default - to add
 * exit conditions and/or other behavior, subclass this class.  The controller calculation and
 * output are performed asynchronously by a separate thread with the period specified by the
 * controller.
 *
 * <p>While this class does more than {@link AsynchronousPIDSubsystem} to ensure
 * thread-safety (as it is a fully usable implementation rather than a base for a user
 * implementation), care should still be taken when using this class to ensure code remains
 * thread-safe.  If you are unfamiliar with thread-safety, consider using {@link
 * SynchronousPIDSubsystem}.
 */
public class AsynchronousPIDCommand extends SendableCommandBase {

  protected final PIDController m_controller;
  protected DoubleSupplier m_measurement;
  protected DoubleSupplier m_setpoint;
  protected DoubleConsumer m_useOutput;
  protected PIDControllerRunner m_runner;

  /**
   * Creates a new AsynchronousPIDCommand, which controls the given output with a PIDController.
   *
   * @param controller        the controller that controls the output
   * @param measurementSource the measurement of the process variable
   * @param setpointSource   the controller's reference (aka setpoint)
   * @param useOutput         the controller's output
   * @param requirements      the subsystems required by this command
   */
  public AsynchronousPIDCommand(PIDController controller,
                                DoubleSupplier measurementSource,
                                DoubleSupplier setpointSource,
                                DoubleConsumer useOutput,
                                Subsystem... requirements) {
    requireNonNull(controller);
    requireNonNull(measurementSource);
    requireNonNull(setpointSource);
    requireNonNull(useOutput);

    m_controller = controller;
    m_measurement = measurementSource;
    m_setpoint = setpointSource;
    m_useOutput = useOutput;
    m_requirements.addAll(Set.of(requirements));
    m_runner = new PIDControllerRunner(m_controller, this::getMeasurement, this::useOutput);
  }

  /**
   * Creates a new AsynchronousPIDCommand, which controls the given output with a PIDController.
   *
   * @param controller        the controller that controls the output.
   * @param measurementSource the measurement of the process variable
   * @param setpoint         the controller's setpoint (aka setpoint)
   * @param useOutput         the controller's output; should be a synchronized method to remain
   *                          threadsafe
   * @param requirements      the subsystems required by this command
   */
  public AsynchronousPIDCommand(PIDController controller,
                                DoubleSupplier measurementSource,
                                double setpoint,
                                DoubleConsumer useOutput,
                                Subsystem... requirements) {
    this(controller, measurementSource, () -> setpoint, useOutput, requirements);
  }

  @Override
  public void initialize() {
    m_controller.reset();
    m_runner.enable();
  }

  @Override
  public void execute() {
    m_controller.setSetpoint(getSetpoint());
  }

  @Override
  public void end(boolean interrupted) {
    m_runner.disable();
  }

  public PIDController getController() {
    return m_controller;
  }

  /**
   * Sets the reference of the PIDController to a constant value.
   *
   * @param reference the reference for the PIDController
   */
  public synchronized void setSetpoint(double reference) {
    m_setpoint = () -> reference;
  }

  /**
   * Sets the reference of the PIDController to the current value plus a specified value.  The set
   * reference will be constant.
   *
   * @param relativeSetpoint the amount by which to increase the reference
   */
  public synchronized void setSetpointRelative(double relativeSetpoint) {
    setSetpoint(m_controller.getSetpoint() + relativeSetpoint);
  }

  /**
   * Gets the reference for the controller.  Wraps the passed-in function so that changes to the
   * function by a subclass are seen by the runner.
   *
   * @return the reference for the controller
   */
  private synchronized double getSetpoint() {
    return m_setpoint.getAsDouble();
  }

  /**
   * Gets the measurement of the process variable.  Wraps the passed-in function so that changes to
   * the function by a subclass are seen by the runner.
   *
   * @return the measurement of the process variable
   */
  private synchronized double getMeasurement() {
    return m_measurement.getAsDouble();
  }

  /**
   * Uses the output of the controller.  Wraps the passed-in function so that changes to the
   * function by a subclass are seen by the runner.
   *
   * @param output the output to use
   */
  private synchronized void useOutput(double output) {
    m_useOutput.accept(output);
  }
}
