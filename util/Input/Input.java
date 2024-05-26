package org.firstinspires.ftc.teamcode.centerstage.util.Input;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Supplier;

import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.concurrent.Executor;
import java.util.concurrent.Executors;

/**
 * This class provides utility methods for handling gamepad input and toggle controls.
 */
public class Input {
    private static Gamepad gamepad1, gamepad2;
    private static Map<KeyCode, Toggle> toggles;
    private static LinearOpMode opMode;
    private static List<Toggle> existingTogglesInOpMode;
    private static Executor executor;
    private static Thread inputUpdaterThread;
    private static InputUpdater inputUpdater;

    static {
        toggles = new HashMap<>();
        existingTogglesInOpMode = new ArrayList<>();
        executor = Executors.newSingleThreadExecutor();
    }

    /**
     * Get the Gamepad1 object.
     *
     * @return The Gamepad1 object.
     */
    public static Gamepad getGamepad1() {
        return gamepad1;
    }

    /**
     * Get the Gamepad2 object.
     *
     * @return The Gamepad2 object.
     */
    public static Gamepad getGamepad2() {
        return gamepad2;
    }

    /**
     * Initialize the Input class with the LinearOpMode instance and gamepad objects.
     *
     * @param opMode_   The LinearOpMode instance.
     * @param gamepad1_ The Gamepad1 object.
     * @param gamepad2_ The Gamepad2 object.
     */
    public static void init(LinearOpMode opMode_, Gamepad gamepad1_, Gamepad gamepad2_) {
        gamepad1 = gamepad1_;
        gamepad2 = gamepad2_;
        existingTogglesInOpMode = new ArrayList<>();
        opMode = opMode_;
        Field[] allFields = opMode.getClass().getDeclaredFields();
        for (Field field : allFields) {
            if (field.getType() == Toggle.class) {
                field.setAccessible(true);
                try {
                    opMode.telemetry.addData("Input init", field.getName());
                    opMode.telemetry.update();
                    Toggle toggle = (Toggle) field.get(opMode);
                    if (toggle != null && toggle.getMapping() != null) {
                        if (field.isAnnotationPresent(UpdateAutomatically.class)) {
                            existingTogglesInOpMode.add(toggle);
                        }
                    }
                } catch (IllegalAccessException e) {
                    opMode.telemetry.addData("Something went wrong", "at Input class init");
                }
            } else if (field.isAnnotationPresent(UpdateAutomatically.class)) {
                throw new RuntimeException("Invalid field for UpdateAutomatically annotation");
            }
        }
        // Add Gamepad1 Toggle instances to the toggles HashMap
        toggles.put(KeyCode.Gamepad1A, new Toggle(KeyCode.Gamepad1A));
        toggles.put(KeyCode.Gamepad1B, new Toggle(KeyCode.Gamepad1B));
        toggles.put(KeyCode.Gamepad1X, new Toggle(KeyCode.Gamepad1X));
        toggles.put(KeyCode.Gamepad1Y, new Toggle(KeyCode.Gamepad1Y));
        toggles.put(KeyCode.Gamepad1DpadUp, new Toggle(KeyCode.Gamepad1DpadUp));
        toggles.put(KeyCode.Gamepad1DpadDown, new Toggle(KeyCode.Gamepad1DpadDown));
        toggles.put(KeyCode.Gamepad1DpadLeft, new Toggle(KeyCode.Gamepad1DpadLeft));
        toggles.put(KeyCode.Gamepad1DpadRight, new Toggle(KeyCode.Gamepad1DpadRight));
        toggles.put(KeyCode.Gamepad1Start, new Toggle(KeyCode.Gamepad1Start));
        toggles.put(KeyCode.Gamepad1Options, new Toggle(KeyCode.Gamepad1Options));
        toggles.put(KeyCode.Gamepad1LeftBumper, new Toggle(KeyCode.Gamepad1LeftBumper));
        toggles.put(KeyCode.Gamepad1RightBumper, new Toggle(KeyCode.Gamepad1RightBumper));

        // Add Gamepad2 Toggle instances to the toggles HashMap
        toggles.put(KeyCode.Gamepad2A, new Toggle(KeyCode.Gamepad2A));
        toggles.put(KeyCode.Gamepad2B, new Toggle(KeyCode.Gamepad2B));
        toggles.put(KeyCode.Gamepad2X, new Toggle(KeyCode.Gamepad2X));
        toggles.put(KeyCode.Gamepad2Y, new Toggle(KeyCode.Gamepad2Y));
        toggles.put(KeyCode.Gamepad2DpadUp, new Toggle(KeyCode.Gamepad2DpadUp));
        toggles.put(KeyCode.Gamepad2DpadDown, new Toggle(KeyCode.Gamepad2DpadDown));
        toggles.put(KeyCode.Gamepad2DpadLeft, new Toggle(KeyCode.Gamepad2DpadLeft));
        toggles.put(KeyCode.Gamepad2DpadRight, new Toggle(KeyCode.Gamepad2DpadRight));
        toggles.put(KeyCode.Gamepad2Start, new Toggle(KeyCode.Gamepad2Start));
        toggles.put(KeyCode.Gamepad2LeftBumper, new Toggle(KeyCode.Gamepad2LeftBumper));
        toggles.put(KeyCode.Gamepad2RightBumper, new Toggle(KeyCode.Gamepad2RightBumper));
        toggles.put(KeyCode.Gamepad2Options, new Toggle(KeyCode.Gamepad2Options));


        inputUpdater = new InputUpdater();
        inputUpdaterThread = new Thread(inputUpdater);
        inputUpdaterThread.start();
    }

    /**
     * Update the gamepad controls in a separate thread.
     */
    public static void updateControls() {
//        executor.execute(new UpdateControlsTask(opMode));
        for (Toggle t : existingTogglesInOpMode) {
            t.update();
        }
    }

    private static class UpdateControlsTask implements Runnable {
        OpMode opMode;

        public UpdateControlsTask(OpMode opMode) {
            this.opMode = opMode;
        }

        @Override
        public void run() {
            for (Toggle t : existingTogglesInOpMode) {
                if (t.getMapping() != null) {
                    t.update();
                }
            }
//            for (Toggle t : toggles.values()) {
//                t.update();
//            }
        }
    }

    /**
     * Get the list of Toggle instances.
     *
     * @return NEAR map of KeyCode to Toggle instances.
     */
    public static Map<KeyCode, Toggle> getToggles() {
        return toggles;
    }

    /**
     * Check if a specific KeyCode is pressed.
     *
     * @param key The KeyCode to check.
     * @return True if the KeyCode is pressed, false otherwise.
     */
    public static boolean GetKeyPressed(KeyCode key) {
        return key.getValue();
    }

    /**
     * Check if a specific KeyCode is clicked.
     *
     * @param key The KeyCode to check.
     * @return True if the KeyCode is clicked, false otherwise.
     */
    public static boolean GetKeyClicked(KeyCode key) {
        if (toggles.isEmpty() || toggles.containsKey(key)) return false;
        return Objects.requireNonNull(toggles.get(key)).isClicked();
    }

    /**
     * Check if a specific KeyCode is released.
     *
     * @param key The KeyCode to check.
     * @return True if the KeyCode is released, false otherwise.
     */
    public static boolean GetKeyReleased(KeyCode key) {
        if (toggles.isEmpty() || toggles.containsKey(key)) return false;
        return Objects.requireNonNull(toggles.get(key)).isReleased();
    }

    /**
     * Returns the value of a chosen axis from the gamepad.
     *
     * @param axis What axis to get.
     * @return Value of the axis on the gamepad.
     */
    public static double GetAxis(Axis axis) {
        return axis.getValue();
    }

    /**
     * Returns the power used on the axis (unsigned).
     *
     * @param axis What axis to use.
     * @return The magnitude (unsigned value) of the axis.
     */
    public static double GetAxisMagnitude(Axis axis) {
        return Math.abs(axis.getValue());
    }

    /**
     * This class represents a keycode for gamepad button checks.
     */
    public static class KeyCode {
        private final Supplier<Boolean> returnFunc;

        private KeyCode(Supplier<Boolean> returnFunc) {
            this.returnFunc = returnFunc;
        }

        /**
         * Check if the keycode is pressed.
         *
         * @return True if the keycode is pressed, false otherwise.
         */
        public boolean getValue() {
            return returnFunc.get();
        }

        //region Button constants for Gamepad1
        public static final KeyCode Gamepad1A = new KeyCode(() -> gamepad1.a);
        public static final KeyCode Gamepad1B = new KeyCode(() -> gamepad1.b);
        public static final KeyCode Gamepad1X = new KeyCode(() -> gamepad1.x);
        public static final KeyCode Gamepad1Y = new KeyCode(() -> gamepad1.y);
        public static final KeyCode Gamepad1DpadUp = new KeyCode(() -> gamepad1.dpad_up);
        public static final KeyCode Gamepad1DpadDown = new KeyCode(() -> gamepad1.dpad_down);
        public static final KeyCode Gamepad1DpadLeft = new KeyCode(() -> gamepad1.dpad_left);
        public static final KeyCode Gamepad1DpadRight = new KeyCode(() -> gamepad1.dpad_right);
        public static final KeyCode Gamepad1Start = new KeyCode(() -> gamepad1.start);
        public static final KeyCode Gamepad1Options = new KeyCode(() -> gamepad1.options);
        public static final KeyCode Gamepad1LeftBumper = new KeyCode(() -> gamepad1.left_bumper);
        public static final KeyCode Gamepad1RightBumper = new KeyCode(() -> gamepad1.right_bumper);
        public static final KeyCode Gamepad1LeftStickButton = new KeyCode(() -> gamepad1.left_stick_button);
        public static final KeyCode Gamepad1RightStickButton = new KeyCode(() -> gamepad1.right_stick_button);
        public static final KeyCode Gamepad1PS = new KeyCode(() -> gamepad1.ps);
        //endregion
        //region Button constants for Gamepad2
        public static final KeyCode Gamepad2A = new KeyCode(() -> gamepad2.a);
        public static final KeyCode Gamepad2B = new KeyCode(() -> gamepad2.b);
        public static final KeyCode Gamepad2X = new KeyCode(() -> gamepad2.x);
        public static final KeyCode Gamepad2Y = new KeyCode(() -> gamepad2.y);
        public static final KeyCode Gamepad2DpadUp = new KeyCode(() -> gamepad2.dpad_up);
        public static final KeyCode Gamepad2DpadDown = new KeyCode(() -> gamepad2.dpad_down);
        public static final KeyCode Gamepad2DpadLeft = new KeyCode(() -> gamepad2.dpad_left);
        public static final KeyCode Gamepad2DpadRight = new KeyCode(() -> gamepad2.dpad_right);
        public static final KeyCode Gamepad2Start = new KeyCode(() -> gamepad2.start);
        public static final KeyCode Gamepad2LeftBumper = new KeyCode(() -> gamepad2.left_bumper);
        public static final KeyCode Gamepad2RightBumper = new KeyCode(() -> gamepad2.right_bumper);
        public static final KeyCode Gamepad2Options = new KeyCode(() -> gamepad2.options);
        public static final KeyCode Gamepad2LeftStickButton = new KeyCode(() -> gamepad2.left_stick_button);
        public static final KeyCode Gamepad2RightStickButton = new KeyCode(() -> gamepad2.right_stick_button);
        public static final KeyCode Gamepad2PS = new KeyCode(() -> gamepad2.ps);

        //endregion
    }

    /**
     * This class represents an axis for gamepad input.
     */
    public static class Axis {
        private final Func<Double> returnFunc;

        private Axis(Func<Double> returnFunc) {
            this.returnFunc = returnFunc;
        }

        /**
         * Get the value of the axis.
         *
         * @return The value of the axis.
         */
        public double getValue() {
            return returnFunc.value();
        }

        // Axis constants for Gamepad1
        public static final Axis Gamepad1RightStickX = new Axis(() -> (double) gamepad1.right_stick_x);
        public static final Axis Gamepad1RightStickY = new Axis(() -> (double) gamepad1.right_stick_y);
        public static final Axis Gamepad1LeftStickY = new Axis(() -> (double) gamepad1.left_stick_y);
        public static final Axis Gamepad1LeftStickX = new Axis(() -> (double) gamepad1.left_stick_x);
        public static final Axis Gamepad1RightTrigger = new Axis(() -> (double) gamepad1.right_trigger);
        public static final Axis Gamepad1LeftTrigger = new Axis(() -> (double) gamepad1.left_trigger);

        // Axis constants for Gamepad2
        public static final Axis Gamepad2RightStickX = new Axis(() -> (double) gamepad2.right_stick_x);
        public static final Axis Gamepad2RightStickY = new Axis(() -> (double) gamepad2.right_stick_y);
        public static final Axis Gamepad2LeftStickY = new Axis(() -> (double) gamepad2.left_stick_y);
        public static final Axis Gamepad2LeftStickX = new Axis(() -> (double) gamepad2.left_stick_x);
        public static final Axis Gamepad2RightTrigger = new Axis(() -> (double) gamepad2.right_trigger);
        public static final Axis Gamepad2LeftTrigger = new Axis(() -> (double) gamepad2.left_trigger);
    }

    private static class InputUpdater implements Runnable {
        @Override
        public void run() {
            while (opMode.opModeIsActive() && !opMode.isStopRequested()) {
                Input.updateControls();
                opMode.idle();
            }
            inputUpdaterThread.interrupt();
        }
    }
}
