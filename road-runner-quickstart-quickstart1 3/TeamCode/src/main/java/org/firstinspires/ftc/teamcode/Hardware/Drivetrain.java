package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Drivetrain {

    public DcMotor LR;
    public DcMotor RR;
    public DcMotor RF;
    public DcMotor LF;

    boolean slowForwards, slowBackwards;
    double slowRotateRight, slowRotateLeft;

    public void driving_TeleOp(Gamepad gamepad) {

        double forward = gamepad.right_stick_y;
        double strafe = -gamepad.left_stick_x;
        double rotate = gamepad.right_stick_x;

        slowRotateRight = gamepad.right_trigger;
        slowRotateLeft = gamepad.left_trigger;
        slowBackwards = gamepad.left_bumper;
        slowForwards = gamepad.right_bumper;

        if (slowBackwards) {
            forward = 0.2;
        } else if (slowForwards) {
            forward = -0.2;
        } else {
            forward = gamepad.right_stick_y;
        }

        if (slowRotateRight > 0)
            rotate = slowRotateRight * -0.2;
        else if (slowRotateLeft > 0)
            rotate = slowRotateLeft * 0.2;
        else
            rotate = gamepad.right_stick_x;

        LR.setPower(-forward + strafe + rotate);
        RR.setPower(forward + strafe + rotate);
        RF.setPower(-forward + strafe - rotate);
        LF.setPower(forward + strafe - rotate);

    }

    public void initDrivetrain(HardwareMap hardwareMap){

        LR= hardwareMap.get(DcMotor.class, "LR");
        RR = hardwareMap.get(DcMotor.class, "RR");
        RF = hardwareMap.get(DcMotor.class, "RF");
        LF = hardwareMap.get(DcMotor.class, "LF");

        LR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        LR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        LR.setDirection(DcMotorSimple.Direction.REVERSE);
        RR.setDirection(DcMotorSimple.Direction.REVERSE);
    }
}