package org.firstinspires.ftc.teamcode.Auto;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Detection.ColorVisionProcessor;
import org.firstinspires.ftc.teamcode.Detection.ColorVisionProcessorRight;
import org.firstinspires.ftc.teamcode.Detection.PropDetectionBlueClose;
import org.firstinspires.ftc.teamcode.Detection.PropDetectionRedClose;
import org.firstinspires.ftc.teamcode.Detection.TeamPropPipelineRed;
import org.firstinspires.ftc.teamcode.Hardware.PIDConstants;
import org.firstinspires.ftc.teamcode.OpModes.TeleOp;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.teamcode.RoadRunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.Detection.PropPipeline;
import org.firstinspires.ftc.teamcode.RoadRunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
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

@Autonomous(name ="🟦BlueRightCycle", group = "CENTERSTAGE")

public class BlueRightCycle extends LinearOpMode {

    SampleMecanumDrive drive;
    TeleOp teleop = new TeleOp();
    Pose2d startPose = new Pose2d(-39.2, 64, Math.toRadians(270));

    Trajectory pos1;

    Trajectory posforward;
    Trajectory pos2;

    Trajectory posback;
    Trajectory pos3;
    Trajectory pos10;
    Trajectory pos4;
    Trajectory pos5;

    TrajectorySequence pos3right;
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
    double integralSum = 0;
    double Kp = PIDConstants.Kp;
    double Ki = PIDConstants.Ki;
    double Kd = PIDConstants.Kd;

    ElapsedTime timer = new ElapsedTime();
    private double lastError = 0;

    double cX = 0;
    double cY = 0;
    double width = 0;




    enum Location {
        Left,
        Center,
        Right
    }


    public enum CASE {
        left,
        center,
        right
    }

    int cazzz = 2;
    PropPipeline PropPipeline = new PropPipeline();


    public static double targetPosition = 0;
    TeamPropPipelineRed teamPropPieline = new TeamPropPipelineRed();
    VisionPortal portal;
    ColorVisionProcessorRight processor;
    @Override
    public void runOpMode() throws InterruptedException {

        drive = new SampleMecanumDrive(hardwareMap);
        teleop.lift.initLiftAuto(hardwareMap);
        teleop.intake.intakeinit(hardwareMap);
        drive.setPoseEstimate(startPose);
        CASE cazul = CASE.right;
        cycle = hardwareMap.get(Servo.class, "cycle");
        cycle.setPosition(0.15);
        processor = new ColorVisionProcessorRight();
        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new android.util.Size(1280,720))
                .enableLiveView(true)
                .addProcessor(processor)
                .enableLiveView(true)
                .build();
        processor.setDetectionColor(ColorVisionProcessorRight.DetectionColor.BLUE);

        while (opModeInInit()) {

            telemetry.update();
            telemetry.addData("Detection", processor.getAnalysis());
            telemetry.update();
            telemetry.update();

        }


        if(opModeIsActive() && !isStopRequested()) {

            if(processor.getAnalysis()== ColorVisionProcessorRight.DetectionPosition.LEFT){
                left();
            }

            if(processor.getAnalysis()== ColorVisionProcessorRight.DetectionPosition.CENTER){
                center();
            }

            if(processor.getAnalysis()== ColorVisionProcessorRight.DetectionPosition.RIGHT){
                right();
            }
        }


    }


    public void center(){

        pos1 = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-35, 34.8, Math.toRadians(270)),SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        pos2 = drive.trajectoryBuilder(pos1.end())
                .lineToLinearHeading(new Pose2d(-46, 44, Math.toRadians(270)),SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        pos3 = drive.trajectoryBuilder(pos2.end())
                .lineToLinearHeading(new Pose2d(-48, 9, Math.toRadians(0)),SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        pos4 = drive.trajectoryBuilder(pos3.end())
                .lineToLinearHeading(new Pose2d(30, 9, Math.toRadians(0)),SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        pos5 = drive.trajectoryBuilder(pos4.end())
                .splineToConstantHeading(new Vector2d(52.5, 33.5), Math.toRadians(0))
                .addTemporalMarker(0.001,()->{
                    teleop.lift.preload();
                })
                .addTemporalMarker(2.1,()->{
                    teleop.lift.preloadServo();
                })
                .build();

        pos6 = drive.trajectoryBuilder(pos5.end())
                .lineToLinearHeading(new Pose2d(25, 7, Math.toRadians(0)),SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(0.001,()->{
                    teleop.lift.RetractServo();
                })
                .addTemporalMarker(1.5,()->{
                    teleop.lift.Retract();
                })
                .build();

        pos7 = drive.trajectoryBuilder(pos6.end())
                .lineToLinearHeading(new Pose2d(-57.6, 10.5, Math.toRadians(0)),SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();


        posback = drive.trajectoryBuilder(pos7.end())
                .forward(3)
                .build();
        pos8 = drive.trajectoryBuilder(posback.end())
                .lineToLinearHeading(new Pose2d(41, 10, Math.toRadians(0)),SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(0.3,()->{
                    exit();

                })

                .addTemporalMarker(3,()->{
                    teleop.lift.cycle();
                })
                .build();



        pos9 = drive.trajectoryBuilder(pos8.end())
                .lineToLinearHeading(new Pose2d(51, 33.5, Math.toRadians(0)),SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(1,()->{
                    teleop.lift.preloadServo();
                })
                .build();

        pos10 = drive.trajectoryBuilder(pos9.end())
                .lineToLinearHeading(new Pose2d(43.5, 30, Math.toRadians(0)),SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(0.1,()->{
                    teleop.lift.RetractServo();
                })
                .addTemporalMarker(1.5,()->{
                    teleop.lift.Retract();
                })
                .build();




        drive.followTrajectory(pos1);
        drive.followTrajectory(pos2);
        drive.followTrajectory(pos3);
        drive.followTrajectory(pos4);
        sleep(200);
        drive.followTrajectory(pos5);
        sleep(200);
        teleop.lift.servoPixel.setPower(1);
        sleep(850);
        teleop.intake.intakeUAD.setPosition(0.3);
        drive.followTrajectory(pos6);
        drive.followTrajectory(pos7);
        collect();
        sleep(500);
        stopintake();
        sleep(300);
        teleop.intake.intakeUAD.setPosition(0.32);
        collect();
        sleep(900);
        teleop.intake.intakeUAD.setPosition(0.45);
        collect();
        drive.followTrajectory(pos8);
        exit();
        sleep(100);
        collect();
        sleep(400);
        drive.followTrajectory(pos9);
        teleop.lift.servoPixel.setPower(1);
        sleep(700);
        teleop.lift.RetractServo();
        sleep(200);
        teleop.lift.Retract();
        drive.followTrajectory(pos10);
        sleep(8000);






    }

    public void left(){

        pos1 = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-29, 34.8, Math.toRadians(310)),SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        pos2 = drive.trajectoryBuilder(pos1.end())
                .lineToLinearHeading(new Pose2d(-44, 44, Math.toRadians(270)),SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        pos3 = drive.trajectoryBuilder(pos2.end())
                .lineToLinearHeading(new Pose2d(-46, 8, Math.toRadians(0)),SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        pos4 = drive.trajectoryBuilder(pos3.end())
                .lineToLinearHeading(new Pose2d(30, 8, Math.toRadians(0)),SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        pos5 = drive.trajectoryBuilder(pos4.end())
                .splineToConstantHeading(new Vector2d(52, 40.5), Math.toRadians(0))
                .addTemporalMarker(0.001,()->{
                    teleop.lift.preload();
                })
                .addTemporalMarker(2.1,()->{
                    teleop.lift.preloadServo();
                })
                .build();

        pos6 = drive.trajectoryBuilder(pos5.end())
                .lineToLinearHeading(new Pose2d(25, 6.5, Math.toRadians(0)),SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(0.001,()->{
                    teleop.lift.RetractServo();
                })
                .addTemporalMarker(1.5,()->{
                    teleop.lift.Retract();
                })
                .build();

        pos7 = drive.trajectoryBuilder(pos6.end())
                .lineToLinearHeading(new Pose2d(-57.6, 10.5, Math.toRadians(0)),SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();


        posback = drive.trajectoryBuilder(pos7.end())
                .forward(4)
                .build();
        pos8 = drive.trajectoryBuilder(posback.end())
                .lineToLinearHeading(new Pose2d(41, 6.5, Math.toRadians(0)),SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(0.3,()->{
                    exit();

                })

                .addTemporalMarker(3,()->{
                    teleop.lift.cycle();
                })
                .build();



        pos9 = drive.trajectoryBuilder(pos8.end())
                .lineToLinearHeading(new Pose2d(51, 34.5, Math.toRadians(0)),SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(1,()->{
                    teleop.lift.preloadServo();
                })
                .build();

        pos10 = drive.trajectoryBuilder(pos9.end())
                .lineToLinearHeading(new Pose2d(42.5, 30, Math.toRadians(0)),SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(0.1,()->{
                    teleop.lift.RetractServo();
                })
                .addTemporalMarker(1.5,()->{
                    teleop.lift.Retract();
                })
                .build();




        drive.followTrajectory(pos1);
        drive.followTrajectory(pos2);
        drive.followTrajectory(pos3);
        drive.followTrajectory(pos4);
        sleep(200);
        drive.followTrajectory(pos5);
        sleep(200);
        teleop.lift.servoPixel.setPower(1);
        sleep(850);
        teleop.intake.intakeUAD.setPosition(0.3);
        drive.followTrajectory(pos6);
        drive.followTrajectory(pos7);
        collect();
        sleep(500);
        stopintake();
        sleep(300);
        teleop.intake.intakeUAD.setPosition(0.32);
        collect();
        sleep(900);
        teleop.intake.intakeUAD.setPosition(0.45);
        collect();
        drive.followTrajectory(pos8);
        exit();
        sleep(100);
        collect();
        sleep(400);
        drive.followTrajectory(pos9);
        teleop.lift.servoPixel.setPower(1);
        sleep(700);
        teleop.lift.RetractServo();
        sleep(200);
        teleop.lift.Retract();
        drive.followTrajectory(pos10);
        sleep(8000);






    }

    public void right(){

        pos1 = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-44.8, 43, Math.toRadians(247.8)),SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        pos2 = drive.trajectoryBuilder(pos1.end())
                .lineToLinearHeading(new Pose2d(-35, 46, Math.toRadians(270)),SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        pos3right= drive.trajectorySequenceBuilder(pos2.end())
                .lineToLinearHeading(new Pose2d(-37, 11, Math.toRadians(270)),SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .lineToLinearHeading(new Pose2d(-37, 9, Math.toRadians(0)),SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        pos4 = drive.trajectoryBuilder(pos3right.end())
                .lineToLinearHeading(new Pose2d(30, 9, Math.toRadians(0)),SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        pos5 = drive.trajectoryBuilder(pos4.end())
                .splineToConstantHeading(new Vector2d(52.5, 26), Math.toRadians(0))
                .addTemporalMarker(0.001,()->{
                    teleop.lift.preload();
                })
                .addTemporalMarker(2.1,()->{
                    teleop.lift.preloadServo();
                })
                .build();

        pos6 = drive.trajectoryBuilder(pos5.end())
                .lineToLinearHeading(new Pose2d(25, 7, Math.toRadians(0)),SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(0.001,()->{
                    teleop.lift.RetractServo();
                })
                .addTemporalMarker(1.5,()->{
                    teleop.lift.Retract();
                })
                .build();

        pos7 = drive.trajectoryBuilder(pos6.end())
                .lineToLinearHeading(new Pose2d(-57.8, 9.4 , Math.toRadians(0)),SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();


        posback = drive.trajectoryBuilder(pos7.end())
                .forward(4.1)
                .build();
        pos8 = drive.trajectoryBuilder(posback.end())
                .lineToLinearHeading(new Pose2d(41, 6.5, Math.toRadians(0)),SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(0.3,()->{
                    exit();

                })

                .addTemporalMarker(3,()->{
                    teleop.lift.cycle();
                })
                .build();



        pos9 = drive.trajectoryBuilder(pos8.end())
                .lineToLinearHeading(new Pose2d(50.51, 35.5, Math.toRadians(0)),SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(1,()->{
                    teleop.lift.preloadServo();
                })
                .build();

        pos10 = drive.trajectoryBuilder(pos9.end())
                .lineToLinearHeading(new Pose2d(42.5, 30, Math.toRadians(0)),SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(0.1,()->{
                    teleop.lift.RetractServo();
                })
                .addTemporalMarker(1,()->{
                    teleop.lift.Retract();
                })
                .build();




        drive.followTrajectory(pos1);
        drive.followTrajectory(pos2);
        drive.followTrajectorySequence(pos3right);
        drive.followTrajectory(pos4);
        sleep(200);
        drive.followTrajectory(pos5);
        sleep(200);
        teleop.lift.servoPixel.setPower(1);
        sleep(850);
        teleop.intake.intakeUAD.setPosition(0.3);
        drive.followTrajectory(pos6);
        drive.followTrajectory(pos7);
        collect();
        sleep(500);
        stopintake();
        sleep(300);
        teleop.intake.intakeUAD.setPosition(0.32);
        collect();
        sleep(900);
        teleop.intake.intakeUAD.setPosition(0.45);
        collect();
        drive.followTrajectory(pos8);
        exit();
        sleep(100);
        collect();
        sleep(400);
        drive.followTrajectory(pos9);
        teleop.lift.servoPixel.setPower(1);
        sleep(700);
        teleop.lift.RetractServo();
        sleep(200);
        teleop.lift.Retract();
        drive.followTrajectory(pos10);
        sleep(100);
        sleep(8000);






    }
    public void collect (){
        teleop.intake.intakeMotor.setPower(0.85);
        teleop.intake.intakeMotorRight.setPower(-0.85);
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
    }





}
