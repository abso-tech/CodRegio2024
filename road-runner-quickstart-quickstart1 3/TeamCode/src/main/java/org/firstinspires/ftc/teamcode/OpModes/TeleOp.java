package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.Airplane;
import org.firstinspires.ftc.teamcode.Hardware.Drivetrain;
import org.firstinspires.ftc.teamcode.Hardware.Hanging;
import org.firstinspires.ftc.teamcode.Hardware.Horizontal;
import org.firstinspires.ftc.teamcode.Hardware.Lift;
import org.firstinspires.ftc.teamcode.Hardware.Robot;


@Config
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOP😘", group = "TEST")

public class TeleOp extends LinearOpMode {


    public ElapsedTime timer = new ElapsedTime();

    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    public double lastError = 0;
    private double integralSum = 0;
    public static double Kp = 0.002;
    public static double KpLeft = 0.002;
    public static double targetPosition;


    public boolean liftrequest;
    public boolean retractrequest;
    Drivetrain drive =new Drivetrain();
    public Horizontal intake = new Horizontal();
    public Airplane airplane = new Airplane();
    Hanging hang = new Hanging();
    Servo cycle;
    public Lift lift = new Lift();

    Robot robot= new Robot();
    enum State {
        INIT,
        TRUE,
        FALSE
    }

   public ElapsedTime liftRequest = new ElapsedTime();
   public  ElapsedTime retractRequest = new ElapsedTime();
    TelemetryPacket packet = new TelemetryPacket();

    public void runOpMode() throws InterruptedException {

        while (opModeInInit()) {
            liftRequest = new ElapsedTime();
            retractRequest = new ElapsedTime();
            liftrequest = false;
            retractrequest = false;
            drive.initDrivetrain(hardwareMap);
            lift.initLiftTeleOp(hardwareMap);
            hang.initHang(hardwareMap);
            intake.intakeinit(hardwareMap);
            airplane.initAirplane(hardwareMap);
            cycle = hardwareMap.get(Servo.class, "cycle");
            cycle.setPosition(0.15);
            dashboard.setTelemetryTransmissionInterval(25);
            targetPosition = 0;
            retractRequest.time();
            liftRequest.time();
            waitForStart();

        }
        while (opModeIsActive() && !isStopRequested()) {

            dashboard.sendTelemetryPacket(packet);
            double powerRight = robot.returnPower(targetPosition, lift.liftRight.getCurrentPosition(),Kp,0,0);
            double powerLeft = robot.returnPower(targetPosition, lift.liftLeft.getCurrentPosition(),Kp,0,0);
            lift.liftLeft.setPower(powerLeft);
            lift.liftRight.setPower(powerRight);

            drive.driving_TeleOp(gamepad1);
            robot.GamePlayBoro(gamepad1,gamepad2);
            robot.TeleOpTelemetry();
            dashboard.sendTelemetryPacket(packet);

        }

    }



}