package org.firstinspires.ftc.teamcode.Testers;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Intake", group = "TEST")

public class IntakeTester extends LinearOpMode {

    private final FtcDashboard dashboard = FtcDashboard.getInstance();
    ElapsedTime timer = new ElapsedTime();

    private double lastError = 0;
    private double integralSum = 0;
    boolean transfer=false;
    public static double Kp = 0.01;
    public static double KpLeft = 0.01;

    public double targetvelocityR=0;public double targetvelocityL=0;
    DcMotor intakeMotor;
    DcMotor intakeMotorRight;
    enum State {
        onePixel,
        twoPixel,
        empty,
    }


    public void runOpMode() throws InterruptedException {

       intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor2");
        intakeMotorRight = hardwareMap.get(DcMotorEx.class, "intakeMotorRight2");
        intakeMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        intakeMotorRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        intakeMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotorRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotorRight.setDirection(DcMotorEx.Direction.REVERSE);
        TelemetryPacket packet = new TelemetryPacket();
        dashboard.setTelemetryTransmissionInterval(25);
        targetvelocityR=0;targetvelocityL=0;
        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {

            dashboard.sendTelemetryPacket(packet);
           intakeMotor.setPower(targetvelocityL);
           intakeMotorRight.setPower(targetvelocityR);

            dashboard.sendTelemetryPacket(packet);

            telemetry.update();


        }

    }
    public double returnPower(double reference, double state, double Kp, double Kd, double Ki){
        double error = reference - state;
        integralSum += error * timer.seconds();
        double derivative = (error- lastError) / timer.seconds();
        lastError = error;

        timer.reset();

        double output = (error * Kp) + (derivative * Kd) + (integralSum * Ki);
        return  output;
    }
}