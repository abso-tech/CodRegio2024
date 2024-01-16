package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Airplane;
import org.firstinspires.ftc.teamcode.Hanging;
import org.firstinspires.ftc.teamcode.Drivetrain;
import org.firstinspires.ftc.teamcode.Horizontal;
import org.firstinspires.ftc.teamcode.Lift;


@Config
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOP😘", group = "TEST")

public class TeleOp extends LinearOpMode {


    ElapsedTime timer = new ElapsedTime();

    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    private double lastError = 0;
    private double integralSum = 0;
    public static double Kp = 0.002;
    public static double KpLeft = 0.002;

    boolean slowForwards, slowBackwards;

    double forward, strafe, rotate;
    double  slowRotateRight, slowRotateLeft ;

    public static double targetPosition;



    Drivetrain drive =new Drivetrain();
    public Horizontal intake = new Horizontal();
    Airplane airplane = new Airplane();
    Hanging hang = new Hanging();

    public Lift lift = new Lift();
    enum State {
        INIT,
        TRUE,
        FALSE
    }


    public void runOpMode() throws InterruptedException {
        ElapsedTime liftRequest = new ElapsedTime();
        ElapsedTime retractRequest = new ElapsedTime();
        drive.initDrivetrain(hardwareMap);
        lift.initLiftTeleOp(hardwareMap);
        hang.initHang(hardwareMap);
        intake.intakeinit(hardwareMap);
        airplane.initAirplane(hardwareMap);

        TelemetryPacket packet = new TelemetryPacket();
        dashboard.setTelemetryTransmissionInterval(25);
        targetPosition=0;
        retractRequest.time();
        liftRequest.time();
        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {

            dashboard.sendTelemetryPacket(packet);
            double powerRight = returnPower(targetPosition, lift.liftRight.getCurrentPosition(),Kp,0,0);
            double powerLeft = returnPower(targetPosition, lift.liftLeft.getCurrentPosition(),Kp,0,0);
            lift.liftLeft.setPower(powerLeft);
            lift.liftRight.setPower(powerRight);



            forward = gamepad1.right_stick_y;
            strafe = -gamepad1.left_stick_x;
            rotate = gamepad1.right_stick_x;
            slowBackwards=gamepad1.left_bumper;
            slowForwards=gamepad1.right_bumper;

            if (slowBackwards) {
                forward = 0.15;
            } else if (slowForwards) {
                forward = -0.15;
            } else {
                forward = gamepad1.right_stick_y;
            }

            if (slowRotateRight > 0)
                rotate = slowRotateRight * 0.2;
            else if (slowRotateLeft > 0)
                rotate = slowRotateLeft * -0.2;
            else
                rotate = gamepad1.right_stick_x;

            drive.LR.setPower(-forward + strafe + rotate);
            drive.RR.setPower(forward + strafe + rotate);
            drive.RF.setPower(-forward + strafe - rotate);
            drive.LF.setPower(forward + strafe - rotate);


            if(gamepad1.b){
                targetPosition=550;
            }
            if(gamepad1.dpad_up){
                lift.servoL.setPosition(0.63);
                lift.servoR.setPosition(0.63);
            }
            if(gamepad1.dpad_down){
                lift.servoL.setPosition(0.8);
                lift.servoR.setPosition(0.8);
            }

            if(gamepad1.a){
                targetPosition=-250;

            }

            if(gamepad2.dpad_left){
                airplane.AirplaneLaunch.setPosition(0.3);

            }

            if(gamepad2.dpad_down){
                airplane.AirplaneUAD.setPosition(0.49);

            }

            if(gamepad2.dpad_up){
                hang.LeftHang.setPosition(0);
                hang.RightHang.setPosition(0);

            }
            if(gamepad2.dpad_right){
                hang.LeftHang.setPosition(1);
                hang.RightHang.setPosition(1);

            }

            if(gamepad1.dpad_left){
                intake.intakeMotor.setPower(0);
                intake.intakeMotorRight.setPower(0);
                lift.servoPixel.setPower(0);
            }

            if(gamepad1.touchpad){
                targetPosition=targetPosition+30;
            }

            if(gamepad1.right_trigger>0) {
                intake.intakeMotor.setPower(-1);
                intake.intakeMotorRight.setPower(1);
                lift.servoPixel.setPower(-1);
            }

            if(gamepad1.left_trigger>0){
                intake.intakeMotor.setPower(1);
                intake.intakeMotorRight.setPower(-1);
                lift.servoPixel.setPower(0);
            }

            if(gamepad1.x){
                lift.servoPixel.setPower(1);
            }
            if(gamepad1.y){
                lift.servoPixel.setPower(1);
            }

            packet.put("positionRight", lift.liftLeft.getCurrentPosition());
            packet.put("positionRight", lift.liftRight.getCurrentPosition());
            packet.put("errorRight", lastError);
            packet.put("errorLeft", lastError);
            telemetry.addData("Timer:", timer);
            telemetry.update();

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