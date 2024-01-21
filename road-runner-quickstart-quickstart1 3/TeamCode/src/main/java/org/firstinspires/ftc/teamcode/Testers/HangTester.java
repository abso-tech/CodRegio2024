package org.firstinspires.ftc.teamcode.Testers;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(name = "HangTester", group = "Test")
public class HangTester extends LinearOpMode {


    private final FtcDashboard dashboard = FtcDashboard.getInstance();
    public static double pos=0;

    public Servo servoR;
    public Servo servoL;



    public void runOpMode() throws InterruptedException {

        servoR= hardwareMap.get(Servo.class, "LeftHang");
        servoL= hardwareMap.get(Servo.class, "RightHang");
        servoR.setPosition(pos);
        servoL.setPosition(pos);
        TelemetryPacket packet = new TelemetryPacket();
        dashboard.setTelemetryTransmissionInterval(15);


        waitForStart();

        while (opModeIsActive() && !isStopRequested()){

            servoR.setPosition(pos);
            servoL.setPosition(pos);

            dashboard.sendTelemetryPacket(packet);
            telemetry.addData("Position",servoL.getPosition());
            telemetry.addData("Position",servoR.getPosition());
            telemetry.update();

        }
    }
}
