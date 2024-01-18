package org.firstinspires.ftc.teamcode.Auto;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Drivetrain;
import org.firstinspires.ftc.teamcode.Horizontal;
import org.firstinspires.ftc.teamcode.Lift;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name ="RedRightCycle", group = "CENTERSTAGE")

public class RedRightCycle extends LinearOpMode {


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

    double integralSum = 0;

    ElapsedTime timer = new ElapsedTime();
    private double lastError = 0;

    double cX = 0;
    double cY = 0;
    double width = 0;

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


        while (!isStarted() && !isStopRequested()) {

            waitForStart();

            if(opModeIsActive() && !isStopRequested()) {
                center();

            }


        }
    }



    public void center(){
        pos1 = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(10, -35, Math.toRadians(90)),SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        pos2 = drive.trajectoryBuilder(pos1.end())
                .lineToLinearHeading(new Pose2d(10, -45, Math.toRadians(0)),SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();


        pos3= drive.trajectoryBuilder(pos2.end())
                .lineToLinearHeading(new Pose2d(47.3, -41.2, Math.toRadians(0)),SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        posinspate= drive.trajectoryBuilder(pos3.end())
                .lineToConstantHeading(new Vector2d(25, -41))
                .build();

        pos4= drive.trajectoryBuilder(posinspate.end())
                .lineToLinearHeading(new Pose2d(25, -65.5, Math.toRadians(0)),SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        posint= drive.trajectoryBuilder(pos4.end())
                .lineToLinearHeading(new Pose2d(-61, -64, Math.toRadians(0)),SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        pos5 = drive.trajectoryBuilder(posint.end())
                .lineToLinearHeading(new Pose2d(-62, -36 ,Math.toRadians(0)),SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        postras = drive.trajectoryBuilder(pos5.end())
                .lineToLinearHeading(new Pose2d(-62.5, -37, Math.toRadians(0)),SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        pos6 = drive.trajectoryBuilder(postras.end())
                .lineToLinearHeading(new Pose2d(-59, -40, Math.toRadians(330)),SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        pos7 = drive.trajectoryBuilder(pos6.end())
                .lineToLinearHeading(new Pose2d(-65, -40, Math.toRadians(330)),SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(0.1, () -> {
                    collect();
                })
                .build();
        pospixeli = drive.trajectoryBuilder(pos7.end())
                .lineToLinearHeading(new Pose2d(-62, -40, Math.toRadians(330)),SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(0.1, () -> {
                    collect();
                })
                .build();
        pos8= drive.trajectoryBuilder(pospixeli.end())
                .lineToLinearHeading(new Pose2d(-58, -63, Math.toRadians(0)),SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(0.1, () -> {
                  exit();
                })
                .build();

        pos9= drive.trajectoryBuilder(pos8.end())
                .lineToLinearHeading(new Pose2d(25, -63, Math.toRadians(0)),SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(2, () -> {
                    teleop.lift.cycle();
                })

                .addTemporalMarker(7, () -> {
                    teleop.lift.preloadServo();
                })
                .build();
        posplace= drive.trajectoryBuilder(pos9.end())
                .lineToLinearHeading(new Pose2d(46.9, -43, Math.toRadians(0)),SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))

                .build();

        drive.followTrajectory(pos1);
        sleep(100);
        teleop.lift.preload();
        drive.followTrajectory(pos2);
        teleop.lift.preloadServo();
        drive.followTrajectory(pos3);
        sleep(300);
        teleop.lift.servoPixel.setPower(1);
        sleep(800);
        drive.followTrajectory(posinspate);
        teleop.lift.RetractServo();
        sleep(300);
        teleop.lift.Retract();
        drive.followTrajectory(pos4);
        drive.followTrajectory(posint);
        drive.followTrajectory(pos5);
        drive.followTrajectory(postras);
        sleep(800);
        drive.followTrajectory(pos6);
        collect();
        sleep(500);
        collect();
        sleep(200);
        drive.followTrajectory(pos7);
        sleep(200);
        collect();
        drive.followTrajectory(pospixeli);
        collect();
        sleep(4000);
        exit();
        sleep(500);
        drive.followTrajectory(pos8);
        stopintake();
        drive.followTrajectory(pos9);
        drive.followTrajectory(posplace);
        teleop.lift.servoPixel.setPower(1);
        sleep(2000);
        teleop.lift.RetractServo();
        sleep(800);
        teleop.lift.Retract();




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
