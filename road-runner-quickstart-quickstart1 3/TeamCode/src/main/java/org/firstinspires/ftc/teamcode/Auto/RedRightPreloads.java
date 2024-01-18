package org.firstinspires.ftc.teamcode.Auto;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.CameraFusedPID;
import org.firstinspires.ftc.teamcode.PIDConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.teamcode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name ="🛑RedRightPreloads", group = "CENTERSTAGE")

public class RedRightPreloads extends LinearOpMode {

    SampleMecanumDrive drive;
    TeleOp teleop = new TeleOp();
    Pose2d startPose = new Pose2d(11, -64.5, Math.toRadians(90));

    Trajectory pos1;
    Trajectory pos2;

    Trajectory pos3;

    Trajectory pos4;
    Trajectory pos5;

    Trajectory pos6;
    Trajectory pos7;
    Trajectory pos8;
    Trajectory pos9;
    Trajectory posinspate;
    Trajectory posint;
    Trajectory posplace;
    Trajectory postras;
    Trajectory pospixeli;

    CASE Case;







    Servo cycle;


    public enum CASE {
        left,
        center,
        right
    }


    public static double targetPosition = 0;
    @Override
    public void runOpMode() throws InterruptedException {

        drive = new SampleMecanumDrive(hardwareMap);
        teleop.lift.initLiftAuto(hardwareMap);
        teleop.intake.intakeinit(hardwareMap);
        drive.setPoseEstimate(startPose);
        CASE cazul = CASE.right;
        cycle = hardwareMap.get(Servo.class,"cycle" );
        cycle.setPosition(0.15);


        waitForStart();

        while (!isStarted() && !isStopRequested()) {

            telemetry.update();
            telemetry.update();
            telemetry.update();
            telemetry.update();
            telemetry.update();
            telemetry.update();


        }



        if(opModeIsActive() && !isStopRequested()) {
                right();

            }



    }



    public void center(){
        pos1 = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(8, -35, Math.toRadians(90)),SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        pos2 = drive.trajectoryBuilder(pos1.end())
                .lineToLinearHeading(new Pose2d(5, -46, Math.toRadians(0)),SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();


        pos3= drive.trajectoryBuilder(pos2.end())
                .lineToLinearHeading(new Pose2d(48.3, -41, Math.toRadians(0)),SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        pos4= drive.trajectoryBuilder(pos3.end())
                .lineToLinearHeading(new Pose2d(43, -20, Math.toRadians(0)),SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();




        drive.followTrajectory(pos1);
        sleep(100);
        teleop.lift.preload();
        drive.followTrajectory(pos2);
        teleop.lift.preloadServo();
        drive.followTrajectory(pos3);
        sleep(400);
        teleop.lift.servoPixel.setPower(0.7);
        sleep(2000);
        drive.followTrajectory(pos4);
        teleop.lift.RetractServo();
        sleep(500);
        teleop.lift.Retract();
        teleop.lift.servoPixel.setPower(0);
        sleep(20000);



    }

    public void right(){
        pos1 = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(11, -35, Math.toRadians(45)),SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        pos2 = drive.trajectoryBuilder(pos1.end())
                .lineToLinearHeading(new Pose2d(8, -50, Math.toRadians(0)),SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();


        pos3= drive.trajectoryBuilder(pos2.end())
                .lineToLinearHeading(new Pose2d(47.8, -47.7, Math.toRadians(0)),SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        pos4= drive.trajectoryBuilder(pos3.end())
                .lineToLinearHeading(new Pose2d(42, -15, Math.toRadians(0)),SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();




        drive.followTrajectory(pos1);
        sleep(100);
        teleop.lift.preload();
        drive.followTrajectory(pos2);
        teleop.lift.preloadServo();
        drive.followTrajectory(pos3);
        sleep(400);
        teleop.lift.servoPixel.setPower(0.7);
        sleep(2000);
        drive.followTrajectory(pos4);
        teleop.lift.RetractServo();
        sleep(500);
        teleop.lift.Retract();
        teleop.lift.servoPixel.setPower(0);
        sleep(1500);



    }
    public void left(){
        pos1 = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(0, -37.5, Math.toRadians(134)),SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        pos2 = drive.trajectoryBuilder(pos1.end())
                .lineToLinearHeading(new Pose2d(12, -45, Math.toRadians(0)),SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();


        pos3= drive.trajectoryBuilder(pos2.end())
                .lineToLinearHeading(new Pose2d(49, -33, Math.toRadians(0)),SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        pos4= drive.trajectoryBuilder(pos3.end())
                .lineToLinearHeading(new Pose2d(43, -15, Math.toRadians(0)),SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();




        drive.followTrajectory(pos1);
        sleep(100);
        teleop.lift.preload();
        drive.followTrajectory(pos2);
        teleop.lift.preloadServo();
        drive.followTrajectory(pos3);
        sleep(400);
        teleop.lift.servoPixel.setPower(0.7);
        sleep(2000);
        drive.followTrajectory(pos4);
        teleop.lift.RetractServo();
        sleep(500);
        teleop.lift.Retract();
        teleop.lift.servoPixel.setPower(0);
        sleep(1500);



    }


    public void collect (){
        teleop.intake.intakeMotor.setPower(-1);
        teleop.intake.intakeMotorRight.setPower(1);
        teleop.lift.servoPixel.setPower(-1);
    }
    public void exit (){
        teleop.intake.intakeMotor.setPower(-1);
        teleop.intake.intakeMotorRight.setPower(1);
        teleop.lift.servoPixel.setPower(-1);
    }

    public void stopintake (){
        teleop.intake.intakeMotor.setPower(0);
        teleop.intake.intakeMotorRight.setPower(0);
        teleop.lift.servoPixel.setPower(0);
    }


}
