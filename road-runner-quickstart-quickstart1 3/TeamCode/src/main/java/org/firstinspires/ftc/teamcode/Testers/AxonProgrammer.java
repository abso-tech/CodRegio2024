package org.firstinspires.ftc.teamcode.Testers;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(name = "AxonProgrammer", group = "Test")
public class AxonProgrammer extends LinearOpMode {


    private final FtcDashboard dashboard = FtcDashboard.getInstance();
    public static double UADPos=0;
    public static double LaunchPos=0;

    public Servo servoR;
    public Servo servoL;



    public void runOpMode() throws InterruptedException {

        servoR= hardwareMap.get(Servo.class, "AirplaneUAD");
        servoL= hardwareMap.get(Servo.class, "AirplaneLaunch");
        servoR.setPosition(UADPos);
        servoL.setPosition(LaunchPos);
        TelemetryPacket packet = new TelemetryPacket();
        dashboard.setTelemetryTransmissionInterval(15);


        waitForStart();

        while (opModeIsActive() && !isStopRequested()){

            servoR.setPosition(UADPos);
            servoL.setPosition(LaunchPos);

            dashboard.sendTelemetryPacket(packet);
            telemetry.addData("Position",servoL.getPosition());
            telemetry.addData("Position",servoR.getPosition());
            telemetry.update();

        }
    }
}
