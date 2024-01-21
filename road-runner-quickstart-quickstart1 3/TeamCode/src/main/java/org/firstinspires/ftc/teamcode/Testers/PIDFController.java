package org.firstinspires.ftc.teamcode.Testers;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@TeleOp(name = "PIDF")
public class PIDFController extends LinearOpMode {

    public DcMotorEx liftRight;
    public DcMotorEx liftLeft;
    ElapsedTime timer = new ElapsedTime();

    private double lastError = 0;
    private double integralSum = 0;

    public static double Kp = 0.002;
    public static double KpLeft = 0.002;

    public static double targetPosition = 0;

    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    @Override
    public void runOpMode() throws InterruptedException {
        TelemetryPacket packet = new TelemetryPacket();

        dashboard.setTelemetryTransmissionInterval(25);
        liftRight = hardwareMap.get(DcMotorEx.class, "liftRight");
        liftLeft = hardwareMap.get(DcMotorEx.class, "liftLeft");

        liftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        liftRight.setDirection(DcMotorEx.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()){

            double powerRight = returnPower(targetPosition, liftRight.getCurrentPosition(), Kp, 0, 0);
            double powerLeft = returnPower(targetPosition, liftLeft.getCurrentPosition(), KpLeft, 0, 0);

            packet.put("powerRight", powerRight);
            packet.put("positionRight", liftRight.getCurrentPosition());
            packet.put("errorRight", lastError);



            packet.put("powerLeft", powerLeft);
            packet.put("positionLeft", liftLeft.getCurrentPosition());
            packet.put("errorLeft", lastError);
            packet.put("ERROR",liftLeft.getCurrentPosition()-liftRight.getCurrentPosition());

            liftRight.setPower(powerRight);
            liftLeft.setPower(powerLeft);

            dashboard.sendTelemetryPacket(packet);
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
