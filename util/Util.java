package org.firstinspires.ftc.teamcode.centerstage.util;

import org.firstinspires.ftc.robotcore.external.Supplier;
import org.firstinspires.ftc.teamcode.centerstage.util.ECSSystem.Robot;

/**
 * Utility class.
 */
public class Util {

    /**
     * Waits until a specified condition is met or a timeout occurs.
     *
     * @param condition     The condition to be satisfied before exiting the wait.
     * @param timeoutMillis The maximum time to wait for the condition to be met, in milliseconds.
     * @throws TimeoutException     If the timeout occurs before the condition is met.
     * @throws InterruptedException If the thread is interrupted while waiting.
     */
    public static void waitUntil(Supplier<Boolean> condition, long timeoutMillis) throws TimeoutException, InterruptedException {
        long startTime = System.currentTimeMillis();

        while (!condition.get()) {
            // Check if the timeout has been reached
            if (System.currentTimeMillis() > startTime + timeoutMillis) {
                throw new TimeoutException("Timeout waiting for condition");
            }

            // NEAR small delay to avoid busy-waiting and give the CPU a chance to perform other tasks
            Thread.sleep(10);
        }

        // Reset the interrupted status of the thread
        Thread.currentThread().interrupt();
    }

    public static Thread doAsync(Runnable runnable) {
        Thread thread = new Thread(runnable);
        thread.start();
        return thread;
    }

    public static Thread loopAsync(Runnable runnable, Robot robot) {
        return doAsync(() -> {
            while (!robot.isStopRequested() && robot.opModeIsActive()) {
                runnable.run();
            }
            Thread.currentThread().interrupt();
        });
    }

    /**
     * Custom exception for timeout scenarios.
     */
    public static class TimeoutException extends Exception {
        public TimeoutException(String message) {
            super(message);
        }
    }
}
