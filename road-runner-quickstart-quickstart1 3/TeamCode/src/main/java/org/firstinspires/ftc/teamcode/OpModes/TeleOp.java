package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.Airplane;
import org.firstinspires.ftc.teamcode.Hardware.Hanging;
import org.firstinspires.ftc.teamcode.Hardware.Drivetrain;
import org.firstinspires.ftc.teamcode.Hardware.Horizontal;
import org.firstinspires.ftc.teamcode.Hardware.Lift;


@Config
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOP😘", group = "TEST")

public class TeleOp extends LinearOpMode {


    ElapsedTime timer = new ElapsedTime();

    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    private double lastError = 0;
    private double integralSum = 0;

    public static double Kp = 0.0045; //pid lift

    boolean slowForwards;
    boolean slowBackwards;

    double forward, strafe, rotate;
    double  slowRotateRight, slowRotateLeft ;

    public static double targetPosition;


    boolean liftrequest,retractrequest;
    Drivetrain drive =new Drivetrain();
    public Horizontal intake = new Horizontal();
    public  Airplane airplane = new Airplane();
    Hanging hang = new Hanging();
    public Lift lift = new Lift();
    enum State {
        INIT,
        TRUE,
        FALSE
    }

    boolean ouuttakeon;

    public void runOpMode() throws InterruptedException {
        ElapsedTime liftRequest = new ElapsedTime();
        ElapsedTime retractRequest = new ElapsedTime();
        liftrequest=false;
        retractrequest=false;
        drive.initDrivetrain(hardwareMap);
        lift.initLiftTeleOp(hardwareMap);
        intake.intakeinit(hardwareMap);
        airplane.initAirplane(hardwareMap);
        TelemetryPacket packet = new TelemetryPacket();
        dashboard.setTelemetryTransmissionInterval(25);
        targetPosition=0;
        retractRequest.time();
        liftRequest.time();
        ouuttakeon=false;
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


            if(gamepad2.y && liftrequest==false){
                targetPosition=1600;
                liftRequest.reset();
                liftrequest=true;
            }
            if(liftRequest.milliseconds()>800 && liftrequest){
                lift.servoL.setPosition(0.56);
                lift.servoR.setPosition(0.56);
                liftrequest=false;
            }


            if(gamepad1.y){
                targetPosition=targetPosition-250;
            }

            if(gamepad2.a && retractrequest==false){
                lift.servoL.setPosition(0.75);
                lift.servoR.setPosition(0.75);
                retractrequest=true;
            }
            if(retractRequest.milliseconds()>1500 && retractrequest){
                targetPosition=-50;
                retractrequest=false;
            }

            if(gamepad2.touchpad){
                intake.intakeMotor.setPower(0);
                intake.intakeMotorRight.setPower(0);
                lift.servoPixel.setPower(0);

                intake.intakeUAD.setPosition(0.2);
            }




            if(gamepad1.a){
                targetPosition=300;
            }
            if(gamepad1.right_bumper){
                targetPosition=targetPosition+70;
            }
            if(gamepad1.left_bumper){
                targetPosition=targetPosition-40;
            }

            if(gamepad2.right_trigger>0) {
                intake.intakeMotor.setPower(-1);
                intake.intakeMotorRight.setPower(-1);
                lift.servoPixel.setPower(-1);
                intake.intakeUAD.setPosition(0.42);
            }



            if(gamepad1.dpad_right){
                airplane.AirplaneUAD.setPosition(0.2);
            }
            if(gamepad2.left_trigger>0){
                intake.intakeMotor.setPower(1);
                intake.intakeMotorRight.setPower(1);
                lift.servoPixel.setPower(0);
            }


            if(gamepad2.b){
                lift.servoPixel.setPower(1);
                ouuttakeon=true;
            }

            if(gamepad1.dpad_down){
                airplane.AirplaneUAD.setPosition(0.3);
            }

            if(gamepad1.dpad_left){
                airplane.AirplaneLaunch.setPosition(0);
            }


            if(gamepad2.dpad_down){
               lift.rotation.setPosition(0.49);
            }


            if(gamepad2.dpad_right){
                lift.rotation.setPosition(0.62);

            }


            if(gamepad2.dpad_left){
                lift.rotation.setPosition(0.35);

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