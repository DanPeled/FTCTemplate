package org.firstinspires.ftc.teamcode.centerstage.Systems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.centerstage.Autonomous.AutoFlow;
import org.firstinspires.ftc.teamcode.centerstage.Systems.Arm.Arm;
import org.firstinspires.ftc.teamcode.centerstage.util.ECSSystem.Component;
import org.firstinspires.ftc.teamcode.centerstage.util.ECSSystem.RobotTelemetry;
import org.firstinspires.ftc.teamcode.centerstage.util.ECSSystem.ThreadedComponent;
import org.firstinspires.ftc.teamcode.centerstage.util.Input.Toggle;
import org.firstinspires.ftc.teamcode.centerstage.util.Location;
import org.firstinspires.ftc.teamcode.centerstage.util.MathUtil;

@ThreadedComponent
public class RobotControl extends Component {
    @RobotTelemetry
    public static MultipleTelemetry multipleTelemetry;
    Gamepad gamepad1, gamepad2;
    // Robot components
    DriveClass drive;
    // Telemetry and dashboard
    FtcDashboard dashboard;
    Telemetry dashboardTelemetry;
    // Control variables
    boolean fieldOriented = true;

    @Override
    public void init() {
        gamepad1 = robot.gamepad1;
        gamepad2 = robot.gamepad2;
    }

    @Override
    public void start() {
        drive.resetOrientation(-1); // SET VALUE
    }


    public void setAlliance(AutoFlow.Alliance alliance) {
        robot.alliance = alliance;
        if (alliance == AutoFlow.Alliance.BLUE) {
         
        } else {
           
        }
        drive.resetOrientation(-1); // SET VALUE
    }

    @Override
    public void update() {
        updateDriverJoystick();
        updateTelemetry();
    }

    // Method to update driving controls
    private void updateDriverJoystick() {
        // Boost factor for driving speed
        double boost = (gamepad1.left_trigger > 0.05 || gamepad1.right_trigger > 0.05) ? 1.5 : 0.6;

        // Drive control based on gamepad input
        double y = pow(-gamepad1.left_stick_y) * boost;
        double x = pow(gamepad1.left_stick_x) * boost + fixToBackdrop;
        double turn = pow(gamepad1.right_stick_x / ((gamepad1.left_trigger > 0.05 || gamepad1.right_trigger > 0.05) ? 1 : 1.7)) * boost;
        
        // Perform dpad-specific actions
        else if (gamepad1.dpad_left) drive.setPowerOriented(0, -0.3, 0, fieldOriented);
        else if (gamepad1.dpad_right) drive.setPowerOriented(0, 0.3, 0, fieldOriented);
        else if (gamepad1.dpad_up) drive.setPowerOriented(0.3, 0, 0, fieldOriented);
        else if (gamepad1.dpad_down) drive.setPowerOriented(-0.3, 0, 0, fieldOriented);
        else drive.setPowerOriented(y, x, turn, fieldOriented);
    }

    // Method to initialize robot subsystems
    private void initSystems() {
        telemetry.addData(">", "Init in progress...");
        telemetry.update();
        // Initialize telemetry
        telemetryInit();
        
        drive = robot.addComponent(DriveClass.class, new DriveClass(DriveClass.ROBOT.GLADOS, new Location(-0.9, 0.4404 / 2, 180), DriveClass.USE_ENCODERS | DriveClass.USE_BRAKE,      DriveClass.DriveMode.LEFT));
    }

    // Method to apply power function
    public double pow(double x) {
        return Math.pow(x, 2) * Math.signum(x);
    }

    // Method to handle force quit command
    private void handleForceQuit() {
        if (gamepad1.y && gamepad1.start && gamepad1.options) {
            robot.requestOpModeStop();
        }
    }

    // Method to initialize telemetry
    private void telemetryInit() {
        dashboard = FtcDashboard.getInstance();
        dashboardTelemetry = dashboard.getTelemetry();
        multipleTelemetry = new MultipleTelemetry(dashboardTelemetry, telemetry);
    }

    // Method to update telemetry data
    private void updateTelemetry() {
    // add data
        multipleTelemetry.update();
    }
}
