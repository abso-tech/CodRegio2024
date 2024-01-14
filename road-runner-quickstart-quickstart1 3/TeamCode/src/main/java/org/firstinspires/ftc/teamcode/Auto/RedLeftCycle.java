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

@Autonomous(name ="RedLeftCycle", group = "CENTERSTAGE")

public class RedLeftCycle extends LinearOpMode {


    SampleMecanumDrive drive;
    TeleOp teleop = new TeleOp();
    Pose2d startPose = new Pose2d(-30.7, -64.5, Math.toRadians(90));

    Trajectory pos1;
    Trajectory pos2;

    Trajectory pos3;

    Trajectory pos4;
    Trajectory pos5;

    Trajectory pos6;
    Trajectory pos7;
    Trajectory pos8;
    Trajectory pos9;

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
        cycle.setPosition(0.3);


        while (!isStarted() && !isStopRequested()) {

            waitForStart();

            if(opModeIsActive() && !isStopRequested()) {
                center();

            }


        }
    }



    public void center(){
        pos1 = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-33, -35, Math.toRadians(90)),SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        pos2 = drive.trajectoryBuilder(pos1.end())
                .lineToLinearHeading(new Pose2d(-35, -40, Math.toRadians(0)),SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        pos3 = drive.trajectoryBuilder(pos2.end())
                .lineToLinearHeading(new Pose2d(-58.2, -39, Math.toRadians(0)),SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        pos4 = drive.trajectoryBuilder(pos3.end())
                .lineToLinearHeading(new Pose2d(-50, -40, Math.toRadians(0)),SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        pos5 = drive.trajectoryBuilder(pos4.end())
                .lineToLinearHeading(new Pose2d(-58, -40, Math.toRadians(0)),SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(0.1, () -> {
                     collect();
                })
                .build();
        pos6 = drive.trajectoryBuilder(pos5.end())
                .lineToLinearHeading(new Pose2d(-56, -62, Math.toRadians(0)),SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        pos7= drive.trajectoryBuilder(pos6.end())
                .lineToLinearHeading(new Pose2d(40, -62, Math.toRadians(0)),SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        pos8= drive.trajectoryBuilder(pos7.end())
                .lineToLinearHeading(new Pose2d(52.5, -28, Math.toRadians(0)),SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        pos9= drive.trajectoryBuilder(pos8.end())
                .lineToLinearHeading(new Pose2d(52.5, -38, Math.toRadians(0)),SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();


        drive.followTrajectory(pos1);
        sleep(200);
        drive.followTrajectory(pos2);
        sleep(100);
        drive.followTrajectory(pos3);
        sleep(800);
        cycle.setPosition(0.8);
        sleep(300);
        drive.followTrajectory(pos4);
        cycle.setPosition(0.3);
        sleep(200);
        cycle.setPosition(0.3);
        sleep(200);
        drive.followTrajectory(pos5);
        collect();
        sleep(2000);
        collect();
        sleep(100);
        drive.followTrajectory(pos6);
        teleop.intake.intakeMotor.setPower(1);
        drive.followTrajectory(pos7);
        teleop.intake.intakeMotor.setPower(0);
        drive.followTrajectory(pos8);
        teleop.lift.preload();
        sleep(400);
        teleop.lift.preloadServo();
        sleep(300);
        teleop.lift.servoPixel.setPower(1);
        sleep(250);
        teleop.lift.servoPixel.setPower(0);
        sleep(100);
        drive.followTrajectory(pos9);
        sleep(800);
        teleop.lift.servoPixel.setPower(1);
        sleep(300);






    }


 public void collect (){
     teleop.lift.servoPixel.setPower(-1);
     teleop.intake.intakeMotor.setPower(-1);
     teleop.lift.servoPixel.setPower(-1);
 }


}
