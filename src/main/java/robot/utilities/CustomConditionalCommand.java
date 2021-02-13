/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package robot.utilities;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;


/**
 * A {@link CustomConditionalCommand} is a {@link Command} that starts one of two commands.
 *
 * <p>
 * A {@link CustomConditionalCommand} uses m_condition to determine whether it should run m_onTrue or
 * m_onFalse.
 * </p>
 *
 * <p>
 * A {@link CustomConditionalCommand} adds the proper {@link Command} to the {@link } during
 * {@link CustomConditionalCommand#initialize()} and then {@link CustomConditionalCommand#isFinished()} will
 * return true once that {@link Command} has finished executing.
 * </p>
 *
 * <p>
 * If no {@link Command} is specified for m_onFalse, the occurrence of that condition will be a
 * no-op.
 * </p>
 *
 * @see Command
 *
 */
public abstract class CustomConditionalCommand extends CommandBase {
    /**
     * The Command to execute if {@link CustomConditionalCommand#condition()} returns true.
     */
    private Command m_onTrue;

    /**
     * The Command to execute if {@link CustomConditionalCommand#condition()} returns false.
     */
    private Command m_onFalse;

    /**
     * Stores command chosen by condition.
     */
    private Command m_chosenCommand;

    /**
     * Creates a new CustomConditionalCommand with given onTrue and onFalse Commands.
     *
     * <p>Users of this constructor should also override condition().
     *
     * @param onTrue The Command to execute if {@link CustomConditionalCommand#condition()} returns true
     */
    public CustomConditionalCommand(Command onTrue) {
        this(onTrue, null);
    }

    /**
     * Creates a new CustomConditionalCommand with given onTrue and onFalse Commands.
     *
     * <p>Users of this constructor should also override condition().
     *
     * @param onTrue The Command to execute if {@link CustomConditionalCommand#condition()} returns true
     * @param onFalse The Command to execute if {@link CustomConditionalCommand#condition()} returns false
     */
    public CustomConditionalCommand(Command onTrue, Command onFalse) {
        m_onTrue = onTrue;
        m_onFalse = onFalse;
    }

    /**
     * Creates a new CustomConditionalCommand with given name and onTrue and onFalse Commands.
     *
     * <p>Users of this constructor should also override condition().
     *
     * @param name the name for this command group
     * @param onTrue The Command to execute if {@link CustomConditionalCommand#condition()} returns true
     */
    public CustomConditionalCommand(String name, Command onTrue) {
        this(name, onTrue, null);
    }

    /**
     * Creates a new CustomConditionalCommand with given name and onTrue and onFalse Commands.
     *
     * <p>Users of this constructor should also override condition().
     *
     * @param name the name for this command group
     * @param onTrue The Command to execute if {@link CustomConditionalCommand#condition()} returns true
     * @param onFalse The Command to execute if {@link CustomConditionalCommand#condition()} returns false
     */
    public CustomConditionalCommand(String name, Command onTrue, Command onFalse) {
        super();
        m_onTrue = onTrue;
        m_onFalse = onFalse;
    }

    /**
     * The Condition to test to determine which Command to run.
     *
     * @return true if m_onTrue should be run or false if m_onFalse should be run.
     */
    protected abstract boolean condition();

    /**
     * Calls {@link CustomConditionalCommand#condition()} and runs the proper command.
     */
    
    @Override
    public void initialize() {
        if (condition()) {
            m_chosenCommand = m_onTrue;
        } else {
            m_chosenCommand = m_onFalse;
        }

        if (m_chosenCommand != null) {

            m_chosenCommand.schedule();
        }
        super.initialize();
    }

    @Override
    public synchronized void cancel() {
        if (m_chosenCommand != null && !m_chosenCommand.isFinished()) {
            m_chosenCommand.cancel();
        }

        super.cancel();
    }

    @Override
    public boolean isFinished() {
        if (m_chosenCommand != null) {
            return m_chosenCommand.isFinished();
        } else {
            return true;
        }
    }

}
