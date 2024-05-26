package org.firstinspires.ftc.teamcode.centerstage.util.ECSSystem;

import org.firstinspires.ftc.teamcode.centerstage.Systems.DriveClass;
import org.firstinspires.ftc.teamcode.centerstage.util.Location;
import org.firstinspires.ftc.teamcode.centerstage.util.Util;

public abstract class BaseDrive extends Component {
    protected boolean useDashboardField;

    public abstract void setPower(double x, double y, double turn);

    public abstract void setPowerOriented(double x, double y, double turn, boolean fieldOriented);

    public abstract double getPosX();

    public abstract double getPosY();

    public abstract double getHeading();

    public abstract void resetOrientation(double angle);

    public void stopPower() {
        setPower(0, 0, 0);
    }

    public void turn(double deg, double power) {
        double targetAngle = getHeading() + deg; // zeroAngle
        turnTo(targetAngle, power);
    }

    public double getDeltaHeading(double target) {
        double currentAngle = getHeading();
        double delta = (target - currentAngle) % 360;

        if (delta < -180) {
            delta = delta + 360;
        }
        if (delta > 180) {
            delta = delta - 360;
        }
        return delta;
    }


    public void turnTo(double targetAngle, double targetPower) {
        double delta = getDeltaHeading(targetAngle);
        double s = (delta < 0) ? -1 : 1;
        while (robot.opModeIsActive() && (delta * s > 0)) {

            delta = getDeltaHeading(targetAngle);
            double gain = 0.02;
            double power = gain * delta * targetPower;
            if (Math.abs(power) < 0.1) power = 0.1 * Math.signum(power);

            setPower(0, power, 0);

            robot.telemetry.addData("target", targetAngle);
            robot.telemetry.addData("current", getHeading());
            robot.telemetry.addData("delta", delta);
            robot.telemetry.addData("power", power);
            robot.telemetry.update();
        }
        stopPower();
    }

    public abstract double goToLocation(Location location, DriveClass.GotoSettings settings);

    public abstract double goTo(double x, double y, double targetPower, double targetHeading, double tolerance, double timeout, boolean superSpeed, Runnable onFinish);

    public double goToLocation(Location location, DriveClass.GotoSettings settings, Runnable midwayAction) {
        return goToAndOperate(location, settings, midwayAction);
    }

    protected double goToAndOperate(Location location, DriveClass.GotoSettings settings, Runnable midwayAction) {
        return goToAndOperate(location, settings, midwayAction, null);
    }

    protected double goToAndOperate(Location location, DriveClass.GotoSettings settings, Runnable midwayAction, Runnable onFinish) {
        if (midwayAction == null) {
            midwayAction = () -> {
            };
        }
        Thread thread = Util.loopAsync(midwayAction, robot);
        double goToResult = goTo(location.x, location.y, settings.power, location.angle, settings.tolerance, settings.timeout, settings.noSlowdown, onFinish);
        thread.interrupt();
        return goToResult;
    }
}
