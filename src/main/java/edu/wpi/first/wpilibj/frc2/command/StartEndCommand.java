/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package edu.wpi.first.wpilibj.frc2.command;

import java.util.function.BooleanSupplier;

import static java.util.Objects.requireNonNull;

/**
 * A command that runs a given runnable when it is initalized, and another runnable when it ends.
 * Useful for running and then stopping a motor, or extending and then retracting a solenoid.
 * Has no end condition as-is; either subclass it or use {@link Command#withTimeout(double)} or
 * {@link Command#interruptOn(BooleanSupplier)} to give it one.
 */
public class StartEndCommand extends SendableCommandBase {
  protected final Runnable m_onInit;
  protected final Runnable m_onEnd;

  /**
   * Creates a new StartEndCommand.  Will run the given runnables when the command starts and when
   * it ends.
   *
   * @param onInit       the Runnable to run on command init
   * @param onEnd        the Runnable to run on command end
   * @param requirements the subsystems required by this command
   */
  public StartEndCommand(Runnable onInit, Runnable onEnd, Subsystem... requirements) {
    m_onInit = requireNonNull(onInit);
    m_onEnd = requireNonNull(onEnd);

    addRequirements(requirements);
  }

  @Override
  public void initialize() {
    m_onInit.run();
  }

  @Override
  public void end(boolean interrupted) {
    m_onEnd.run();
  }
}
