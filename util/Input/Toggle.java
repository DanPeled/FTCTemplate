package org.firstinspires.ftc.teamcode.centerstage.util.Input;


import org.firstinspires.ftc.robotcore.external.Supplier;

/**
 * Represents a toggle switch that can be used to track state changes and events.
 */
public class Toggle {

    private boolean lastInput = false, pressed = false, changed = false, state = false;
    private Input.KeyCode mapping = null;
    private Supplier<Boolean> returnFunction;

    /**
     * Default constructor for Toggle.
     */
    public Toggle() {
    }

    /**
     * Constructor for Toggle with a custom return function.
     *
     * @param returnFunction The function to determine the initial state of the toggle.
     */
    public Toggle(Supplier<Boolean> returnFunction) {
        this.returnFunction = returnFunction;
    }

    /**
     * Constructor for Toggle with an initial state.
     *
     * @param initialState The initial state of the toggle.
     */
    public Toggle(boolean initialState) {
        this.state = initialState;
    }

    /**
     * Constructor for Toggle with an assigned KeyCode mapping.
     *
     * @param mapping The KeyCode mapping for the toggle.
     */
    public Toggle(Input.KeyCode mapping) {
        this.mapping = mapping;
    }

    /**
     * Updates the toggle based on the provided input.
     *
     * @param input The new input state.
     */
    public void update(boolean input) {
        if (lastInput != input) {
            changed = true;
            pressed = input;
            if (pressed) {
                state = !state;
            }
            lastInput = input;
        } else {
            changed = false;
        }
    }

    /**
     * Updates the toggle based on the assigned mapping.
     */
    public void update() {
        boolean input = false;
        if (mapping != null) {
            input = Input.GetKeyPressed(mapping);
        }
        update(input);
    }

    /**
     * Sets the toggle state and marks it as changed.
     *
     * @param input The new state of the toggle.
     */
    public void set(boolean input) {
        changed = lastInput != input;
        state = input;
        lastInput = input;
    }

    /**
     * Toggles the current state of the toggle and marks it as changed.
     *
     * @return The new state after toggling.
     */
    public boolean toggle() {
        state = !state;
        changed = true;
        return state;
    }

    /**
     * Checks if the toggle is currently pressed.
     *
     * @return True if the toggle is pressed, false otherwise.
     */
    public boolean isPressed() {
        return pressed;
    }

    /**
     * Checks if the toggle state has changed.
     *
     * @return True if the state has changed, false otherwise.
     */
    public boolean isChanged() {
        return changed;
    }

    /**
     * Gets the current state of the toggle.
     *
     * @return The current state of the toggle.
     */
    public boolean getState() {
        return state;
    }

    /**
     * Checks if the toggle has been clicked (changed to pressed state).
     *
     * @return True if the toggle has been clicked, false otherwise.
     */
    public boolean isClicked() {
        return changed && pressed;
    }

    /**
     * Checks if the toggle has been released (changed to released state).
     *
     * @return True if the toggle has been released, false otherwise.
     */
    public boolean isReleased() {
        return changed && !pressed;
    }

    /**
     * Gets the KeyCode mapping assigned to this toggle.
     *
     * @return The KeyCode mapping.
     */
    public Input.KeyCode getMapping() {
        return mapping;
    }

    /**
     * Remaps the toggle to a new KeyCode.
     *
     * @param newMapping The new KeyCode mapping.
     */
    public void remap(Input.KeyCode newMapping) {
        this.mapping = newMapping;
    }
}
