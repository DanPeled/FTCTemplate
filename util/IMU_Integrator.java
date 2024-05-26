package org.firstinspires.ftc.teamcode.centerstage.util;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.opencv.core.Point;

import java.util.ArrayList;
import java.util.Arrays;


public class IMU_Integrator implements BNO055IMU.AccelerationIntegrator {

	final double tile = 0.6;
	volatile private DcMotorEx fl = null;
	volatile private DcMotorEx fr = null;
	volatile private DcMotorEx bl = null;
	volatile private DcMotorEx br = null;

	private int fl_startPos = 0;
	private int fr_startPos = 0;
	private int bl_startPos = 0;
	private int br_startPos = 0;

	private double forwardTicksPerMeter;
	private double strafeTicksPerMeter;

	private boolean useDashBoard;
    final double meters_to_inches = 39.37008;
    private ArrayList<Double> pathx;
    private ArrayList<Double> pathy;
	private long lastTimestamp = 0;

	private Point origin; // origin point of action
	private Point direction; // x,y direction for dashboard

	private double angularOffset = 0;


	private BNO055IMU imu = null;
	BNO055IMU.Parameters parameters = null;

	Position position = new Position();
	Velocity velocity = new Velocity();
	Acceleration acceleration = null;

	public Position getPosition() {
		return this.position;
	}
	public Velocity getVelocity() {
		return this.velocity;
	}
	public Acceleration getAcceleration() {
		return this.acceleration;
	}

	private class FS {
		public double f = 0;
		public double s = 0;
		FS(double f, double s) {
			this.f = f;
			this.s = s;
		}
	}

	public  double getX() { return position.x; }
	public  double getY() { return position.y; }

	public IMU_Integrator(BNO055IMU imu, HardwareMap hw, double forwardTicksPerMeter, double strafeTicksPerMeter, boolean useDahsboard, Point origin, Point direction, double angularOffset) {
		this.imu = imu;
		// Constructor
		fl = hw.get(DcMotorEx.class, "fl");
		fr = hw.get(DcMotorEx.class, "fr");
		bl = hw.get(DcMotorEx.class, "bl");
		br = hw.get(DcMotorEx.class, "br");
		this.origin = origin;
		this.direction = direction;
		this.forwardTicksPerMeter = forwardTicksPerMeter;
		this.strafeTicksPerMeter = strafeTicksPerMeter;
		this.useDashBoard = useDahsboard;

		if (this.useDashBoard) {
			this.pathx = new ArrayList<>();
			this.pathy = new ArrayList<>();
		}
		this.angularOffset = angularOffset;
	}

	public FS getDeltaDistance() {
		int fl_tick = fl.getCurrentPosition();
		int fr_tick = fr.getCurrentPosition();
		int bl_tick = bl.getCurrentPosition();
		int br_tick = br.getCurrentPosition();

		double fl_dist = fl_tick - fl_startPos;
		double fr_dist = fr_tick - fr_startPos;
		double bl_dist = bl_tick - bl_startPos;
		double br_dist = br_tick - br_startPos;


		double f = ( bl_dist + br_dist + fr_dist + fl_dist) / forwardTicksPerMeter / 4;
		double s = (-bl_dist + br_dist - fr_dist + fl_dist) / strafeTicksPerMeter / 4;

		fl_startPos = fl_tick;
		fr_startPos = fr_tick;
		bl_startPos = bl_tick;
		br_startPos = br_tick;

		return new FS(f,s);
	}

	public void resetPosition() {
		position.x = 0;
		position.y = 0;
	}

	public void initialize(BNO055IMU.Parameters parameters, Position initialPosition, Velocity initialVelocity) {
		this.parameters = parameters;
		this.position = initialPosition != null ? initialPosition : this.position;

		this.velocity = initialVelocity != null ? initialVelocity : this.velocity;
		this.acceleration = null;


		if (this.useDashBoard) {
			Point p = transformDashboard(this.position);
			this.pathx.add(p.x);
			this.pathy.add(p.y);
		}
	}

	public Point transformDashboard(Position pos) {
		double x = Math.signum(direction.x) * (pos.x + origin.x) * meters_to_inches;
		double y = Math.signum(direction.y) * (pos.y + origin.y) * meters_to_inches;
//		double z = (pos.z + origin_offset.z) * meters_to_inches;
		return new Point(x,y);
	}

	public double getHeading() {
		Orientation orientation = imu.getAngularOrientation();
		return -orientation.firstAngle + angularOffset;
	}

	@Override
	public void update(Acceleration linearAcceleration) {

		if (this.useDashBoard && FtcDashboard.getInstance() != null && pathx.size() > 2) {
			Telemetry t = FtcDashboard.getInstance().getTelemetry();
			t.addData("[-2]", Arrays.toString(new double[]{pathx.get(pathx.size() - 2), pathy.get(pathy.size() - 2)}));
			t.addData("[-1]", Arrays.toString(new double[]{pathx.get(pathx.size() - 1), pathy.get(pathy.size() - 1)}));
		}


		FS delta = getDeltaDistance();

		double a = -getHeading() / 180.0 * Math.PI;

		this.position.x += delta.s * Math.cos(a) - delta.f * Math.sin(a);
		this.position.y += delta.f * Math.cos(a) + delta.s * Math.sin(a);

		// 100000000000 ps = 100 ms = 0.1 s
		if (this.useDashBoard && linearAcceleration.acquisitionTime - this.lastTimestamp >= 5000000L) {
			Point p = transformDashboard(this.position); // transform
			double lastx = pathx.get(pathx.size() - 1); // last path x
	        double lasty = pathy.get(pathy.size() - 1); // last path y
			if (Math.abs(lastx - p.x) > 1 || Math.abs(lasty - p.y) > 1) {
	        	pathx.add(p.x);
	            pathy.add(p.y);

	            TelemetryPacket packet = new TelemetryPacket();
	            Canvas canvas = packet.fieldOverlay();

	            canvas.setStroke("tomato");
	            canvas.strokePolyline(to_d_katan(pathx), to_d_katan(pathy));
	            Point o = transformDashboard(new Position()); // transform 0,0
	            canvas.fillCircle(o.x, o.y, 3);

	            FtcDashboard.getInstance().sendTelemetryPacket(packet);
	        }

			this.lastTimestamp = linearAcceleration.acquisitionTime;
		}

//		FtcDashboard.getInstance().getTelemetry().addData("pox", this.position.x);
//		FtcDashboard.getInstance().getTelemetry().addData("poy", this.position.y);
//		FtcDashboard.getInstance().getTelemetry().update();

//		if (linearAcceleration.acquisitionTime != 0L) {
//			if (this.acceleration != null) {
//				Acceleration accelPrev = this.acceleration;
//				Velocity velocityPrev = this.velocity;
//				this.acceleration = linearAcceleration;
//				if (accelPrev.acquisitionTime != 0L) {
//					Velocity deltaVelocity = NavUtil.meanIntegrate(this.acceleration, accelPrev);
//					this.velocity = NavUtil.plus(this.velocity, deltaVelocity);
//				}
//
//				if (velocityPrev.acquisitionTime != 0L) {
//					Position deltaPosition = NavUtil.meanIntegrate(this.velocity, velocityPrev);
//					this.position = NavUtil.plus(this.position, deltaPosition);
//				}
//
//				if (this.parameters != null && this.parameters.loggingEnabled) {
//					RobotLog.vv(this.parameters.loggingTag, "dt=%.3fs accel=%s vel=%s pos=%s", new Object[]{(double)(this.acceleration.acquisitionTime - accelPrev.acquisitionTime) * 1.0E-9D, this.acceleration, this.velocity, this.position});
//				}
//			} else {
//				this.acceleration = linearAcceleration;
//			}
//		}

	}

	private double[] to_d_katan(ArrayList<Double> arr) {
		double[] a = new double[arr.size()];
		for (int i = 0; i < arr.size(); i++) {
			a[i] = arr.get(i);
		}

		return a;
	}

}
