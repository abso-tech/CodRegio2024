package org.firstinspires.ftc.teamcode.Auto;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Detection.ColorVisionProcessor;
import org.firstinspires.ftc.teamcode.Detection.PropDetectionRedFar;
import org.firstinspires.ftc.teamcode.Detection.TeamPropPipelineRed;
import org.firstinspires.ftc.teamcode.Hardware.PIDConstants;
import org.firstinspires.ftc.teamcode.OpModes.TeleOp;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Detection.PropPipeline;

import org.firstinspires.ftc.teamcode.RoadRunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.RoadRunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.openftc.easyopencv.OpenCvCamera;


@Autonomous(name ="🛑RedLeft", group = "CENTERSTAGE")

public class RedLeft extends LinearOpMode {

    SampleMecanumDrive drive;
    TeleOp teleop = new TeleOp();
    Pose2d startPose = new Pose2d(-39.2, -64, Math.toRadians(90));

    Trajectory pos1;
    Trajectory pos2;

    Trajectory pos3;

    Trajectory pos4;
    Trajectory pos5;

    Trajectory pos6;
    Trajectory pos7;
    Trajectory posback;
    Trajectory pos8;

    TrajectorySequence pos3right;

    Trajectory pos10;
    Trajectory pos9;
    Trajectory posinspate;
    Trajectory posint;
    Trajectory posplace;
    Trajectory postras;
    Trajectory pospixeli;

    CASE Case;

    double integralSum = 0;
    double Kp = PIDConstants.Kp;
    double Ki = PIDConstants.Ki;
    double Kd = PIDConstants.Kd;

    ElapsedTime timer = new ElapsedTime();
    private double lastError = 0;

    double cX = 0;
    double cY = 0;
    double width = 0;

    private OpenCvCamera controlHubCam;  // Use OpenCvCamera class from FTC SDK
    /** MAKE SURE TO CHANGE THE FOV AND THE RESOLUTIONS ACCORDINGLY **/
    private static final int CAMERA_WIDTH = 640; // width  of wanted camera resolution
    private static final int CAMERA_HEIGHT = 360; // height of wanted camera resolution
    private static final double FOV = 40;

    // Calculate the distance using the formula
    public static final double objectWidthInRealWorldUnits = 3.75;  // Replace with the actual width of the object in real-world units
    public static final double focalLength = 728;  // Replace with the focal length of the camera in pixels


    enum Location{
        Left,
        Center,
        Right
    }




    public enum CASE {
        left,
        center,
        right
    }

    int cazzz=2;
    private final FtcDashboard dashboard = FtcDashboard.getInstance();
    PropPipeline PropPipeline = new PropPipeline();


    public static double targetPosition = 0;
    TeamPropPipelineRed teamPropPieline = new TeamPropPipelineRed();
    VisionPortal portal;
    PropDetectionRedFar processor;
    @Override
    public void runOpMode() throws InterruptedException {

        drive = new SampleMecanumDrive(hardwareMap);
        teleop.lift.initLiftAuto(hardwareMap);
        teleop.intake.intakeinit(hardwareMap);
        teleop.airplane.initAirplane(hardwareMap);
        drive.setPoseEstimate(startPose);
        CASE cazul = CASE.right;
        processor = new PropDetectionRedFar();
        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new android.util.Size(1280, 720))
                .enableLiveView(true)
                .addProcessor(processor)
                .enableLiveView(true)
                .build();


        while (opModeInInit()) {
            telemetry.addData("case",processor.detection);
            telemetry.addData("right",processor.rightSum);
            telemetry.addData("middle",processor.middleSum);
            dashboard.setTelemetryTransmissionInterval(55);
            telemetry.update();

        }

        waitForStart();

        if (opModeIsActive() && !isStopRequested()) {

            if(processor.detection==1){
                left();
            }

            if(processor.detection==2){
                center();
            }

            if(processor.detection==3){
                right();
            }
        }



    }


    public void center(){

        pos1 = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-37, -35, Math.toRadians(90)),SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        pos2 = drive.trajectoryBuilder(pos1.end())
                .lineToLinearHeading(new Pose2d(-50, -44, Math.toRadians(90)),SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        pos3 = drive.trajectoryBuilder(pos2.end())
                .lineToLinearHeading(new Pose2d(-48, -17, Math.toRadians(0)),SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        pos4 = drive.trajectoryBuilder(pos3.end())
                .lineToLinearHeading(new Pose2d(29, -17, Math.toRadians(0)),SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        pos5 = drive.trajectoryBuilder(pos4.end())
                .splineToConstantHeading(new Vector2d(51.4, -41), Math.toRadians(0))
                .addTemporalMarker(0.001,()->{
                    teleop.lift.preload();
                })
                .addTemporalMarker(2.2,()->{
                    teleop.lift.preloadServo();
                })
                .build();

        pos6 = drive.trajectoryBuilder(pos5.end())
                .lineToLinearHeading(new Pose2d(25, -14, Math.toRadians(0)),SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(0.001,()->{
                    teleop.lift.RetractServo();
                })
                .addTemporalMarker(1.5,()->{
                    teleop.lift.Retract();
                })
                .build();

        pos7 = drive.trajectoryBuilder(pos6.end())
                .lineToLinearHeading(new Pose2d(-61.1, -15.5, Math.toRadians(0)),SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();


        posback = drive.trajectoryBuilder(pos7.end())
                .forward(4)
                .build();
        pos8 = drive.trajectoryBuilder(posback.end())
                .lineToLinearHeading(new Pose2d(41, -12, Math.toRadians(0)),SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(0.2,()->{
                    exit();

                })

                .addTemporalMarker(3.3,()->{
                   teleop.lift.cycle();
                })
                .build();



        pos9 = drive.trajectoryBuilder(pos8.end())
                .lineToLinearHeading(new Pose2d(49, -38.5, Math.toRadians(0)),SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(1,()->{
                    teleop.lift.preloadServo();
                })
                .build();

        pos10 = drive.trajectoryBuilder(pos9.end())
                .lineToLinearHeading(new Pose2d(41, -30, Math.toRadians(0)),SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
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
        sleep(700);
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
                .lineToLinearHeading(new Pose2d(-31.7, -35.78, Math.toRadians(38)),SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        pos2 = drive.trajectoryBuilder(pos1.end())
                .lineToLinearHeading(new Pose2d(-46, -44, Math.toRadians(90)),SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        pos3 = drive.trajectoryBuilder(pos2.end())
                .lineToLinearHeading(new Pose2d(-44, -16, Math.toRadians(0)),SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();


        pos4 = drive.trajectoryBuilder(pos3.end())
                .lineToLinearHeading(new Pose2d(29.8, -16, Math.toRadians(0)),SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        pos5 = drive.trajectoryBuilder(pos4.end())
                .splineToConstantHeading(new Vector2d(51.6, -49), Math.toRadians(0))
                .addTemporalMarker(0.001,()->{
                    teleop.lift.preload();
                })
                .addTemporalMarker(2.2,()->{
                    teleop.lift.preloadServo();
                })
                .build();

        pos6 = drive.trajectoryBuilder(pos5.end())
                .lineToLinearHeading(new Pose2d(25, -14.5, Math.toRadians(0)),SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(0.001,()->{
                    teleop.lift.RetractServo();
                })
                .addTemporalMarker(1.5,()->{
                    teleop.lift.Retract();
                })
                .build();

        pos7 = drive.trajectoryBuilder(pos6.end())
                .lineToLinearHeading(new Pose2d(-60.7, -17.3, Math.toRadians(0)),SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();


        posback = drive.trajectoryBuilder(pos7.end())
                .forward(4)
                .build();
        pos8 = drive.trajectoryBuilder(posback.end())
                .lineToLinearHeading(new Pose2d(37.8, -14.5, Math.toRadians(0)),SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(0.3,()->{
                    exit();

                })

                .addTemporalMarker(3,()->{
                    teleop.lift.cycle();
                })
                .build();



        pos9 = drive.trajectoryBuilder(pos8.end())
                .lineToLinearHeading(new Pose2d(48.44, -39.5, Math.toRadians(0)),SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(1,()->{
                    teleop.lift.preloadServo();
                })
                .build();

        pos10 = drive.trajectoryBuilder(pos9.end())
                .lineToLinearHeading(new Pose2d(41, -30, Math.toRadians(0)),SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
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
        sleep(700);
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
                .lineToLinearHeading(new Pose2d(-47, -39.12, Math.toRadians(90)),SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        pos2 = drive.trajectoryBuilder(pos1.end())
                .lineToLinearHeading(new Pose2d(-34.4, -46, Math.toRadians(90)),SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        pos3right= drive.trajectorySequenceBuilder(pos2.end())
                .lineToLinearHeading(new Pose2d(-35, -11, Math.toRadians(90)),SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .lineToLinearHeading(new Pose2d(-35, -11.8, Math.toRadians(0)),SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();



        pos4 = drive.trajectoryBuilder(pos3right.end())
                .lineToLinearHeading(new Pose2d(30, -16, Math.toRadians(0)),SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        pos5 = drive.trajectoryBuilder(pos4.end())
                .splineToConstantHeading(new Vector2d(51.6 , -33.7), Math.toRadians(0))
                .addTemporalMarker(0.001,()->{
                    teleop.lift.preload();
                })
                .addTemporalMarker(2.2,()->{
                    teleop.lift.preloadServo();
                })
                .build();

        pos6 = drive.trajectoryBuilder(pos5.end())
                .lineToLinearHeading(new Pose2d(25, -14, Math.toRadians(0)),SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(0.001,()->{
                    teleop.lift.RetractServo();
                })
                .addTemporalMarker(1.5,()->{
                    teleop.lift.Retract();
                })
                .build();

        pos7 = drive.trajectoryBuilder(pos6.end())
                .lineToLinearHeading(new Pose2d(-60.1, -15.2, Math.toRadians(0)),SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();


        posback = drive.trajectoryBuilder(pos7.end())
                .forward(4)
                .build();
        pos8 = drive.trajectoryBuilder(posback.end())
                .lineToLinearHeading(new Pose2d(41, -13, Math.toRadians(0)),SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(0.3,()->{
                    exit();

                })

                .addTemporalMarker(3,()->{
                    teleop.lift.cycle();
                })
                .build();



        pos9 = drive.trajectoryBuilder(pos8.end())
                .lineToLinearHeading(new Pose2d(49, -39.5, Math.toRadians(0)),SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(1,()->{
                    teleop.lift.preloadServo();
                })
                .build();

        pos10 = drive.trajectoryBuilder(pos9.end())
                .lineToLinearHeading(new Pose2d(41, -30, Math.toRadians(0)),SampleMecanumDrive.getVelocityConstraint(60, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addTemporalMarker(0.1,()->{
                    teleop.lift.RetractServo();
                })
                .addTemporalMarker(1.5,()->{
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
        sleep(700);
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




    public void collect (){
        teleop.intake.intakeMotor.setPower(0.9);
        teleop.intake.intakeMotorRight.setPower(-0.9);
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
