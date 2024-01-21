package org.firstinspires.ftc.teamcode.Testers;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "LiftTester", group = "TEST")

public class LiftTESTER extends LinearOpMode {

    private final FtcDashboard dashboard = FtcDashboard.getInstance();
    ElapsedTime timer = new ElapsedTime();

    private double lastError = 0;
    private double integralSum = 0;
    boolean transfer=false;
    public static double Kp = 0.01;
    public static double KpLeft = 0.01;

    public double targetPosition=0;
    DcMotor liftLeft;
    DcMotor liftRight;
    enum State {
        onePixel,
        twoPixel,
        empty,
    }


    public void runOpMode() throws InterruptedException {

        liftLeft = hardwareMap.get(DcMotorEx.class, "liftLeft");
        liftRight = hardwareMap.get(DcMotorEx.class, "liftRight");
        liftLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        liftRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        liftLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        liftRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        liftRight.setDirection(DcMotorEx.Direction.REVERSE);
        TelemetryPacket packet = new TelemetryPacket();
        dashboard.setTelemetryTransmissionInterval(25);
        targetPosition=0;
        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {

            dashboard.sendTelemetryPacket(packet);
            double powerRight = returnPower(targetPosition, liftLeft.getCurrentPosition(),Kp,0,0);
            double powerLeft = returnPower(targetPosition, liftRight.getCurrentPosition(),Kp,0,0);
            liftLeft.setPower(powerRight);
            liftRight.setPower(powerLeft);


            if(gamepad1.b){
                targetPosition=80;
            }

            if(gamepad1.a){
                targetPosition=10;
            }
            dashboard.sendTelemetryPacket(packet);

            packet.put("Lift",liftLeft.getPower());
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