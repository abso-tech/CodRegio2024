package org.firstinspires.ftc.teamcode.Hardware;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.OpModes.TeleOp;

public class Robot {

    public ElapsedTime timer = new ElapsedTime();

    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    public double lastError = 0;
    private double integralSum = 0;
    public static double Kp = 0.002;
    public static double KpLeft = 0.002;
    public static double targetPosition;
    Hanging hang;
    Lift lift;
    Horizontal intake;
    Drivetrain DriveTeleOp;
    Airplane airplane;

    TeleOp teleop;

    public double returnPower(double reference, double state, double Kp, double Kd, double Ki){
        double error = reference - state;
        integralSum += error * timer.seconds();
        double derivative = (error- lastError) / timer.seconds();
        lastError = error;

        timer.reset();

        double output = (error * Kp) + (derivative * Kd) + (integralSum * Ki);
        return  output;
    }
    public void TeleOpTelemetry(){
        telemetry.addData("positionRight", lift.liftLeft.getCurrentPosition());
        telemetry.addData("positionRight", lift.liftRight.getCurrentPosition());
        telemetry.addData("errorRight", teleop.lastError);
        telemetry.addData("errorLeft", teleop.lastError);
        telemetry.addData("Timer:", teleop.timer);
        telemetry.update();

    }
    public void GamePlayBoro(Gamepad gamepad1,Gamepad gamepad2){
        if(gamepad2.y && teleop.liftrequest==false){
            teleop.targetPosition=550;
            teleop.liftRequest.reset();
            teleop.liftrequest=true;
        }
        if(teleop.liftRequest.milliseconds()>800 && teleop.liftrequest){
            lift.servoL.setPosition(0.63);
            lift.servoR.setPosition(0.63);
            teleop.liftrequest=false;
        }
        if(teleop.retractRequest.milliseconds()>2000 && teleop.retractrequest){
            teleop.targetPosition=-250;
            teleop.retractrequest=false;
        }

        if(gamepad2.a && teleop.retractrequest==false){
            lift.servoL.setPosition(0.828);
            lift.servoR.setPosition(0.828);
            teleop.retractrequest=true;
        }


        if(gamepad2.touchpad){
            intake.intakeMotor.setPower(0);
            intake.intakeMotorRight.setPower(0);
            lift.servoPixel.setPower(0);
        }

        if(gamepad1.touchpad){
            teleop.targetPosition=teleop.targetPosition+30;
        }

        if(gamepad2.right_trigger>0) {
            intake.intakeMotor.setPower(-1);
            intake.intakeMotorRight.setPower(1);
            lift.servoPixel.setPower(-1);
        }

        if(gamepad2.left_trigger>0){
            intake.intakeMotor.setPower(1);
            intake.intakeMotorRight.setPower(-1);
            lift.servoPixel.setPower(0);
        }


        if(gamepad2.b){
            lift.servoPixel.setPower(1);
        }

        if(gamepad1.dpad_down){
            airplane.AirplaneUAD.setPosition(0.25);
        }

        if(gamepad1.dpad_left){
            airplane.AirplaneLaunch.setPosition(0.5);
        }

        if(gamepad2.dpad_up){
            hang.LeftHang.setPosition(0);
            hang.RightHang.setPosition(0);
        }
    }

}
