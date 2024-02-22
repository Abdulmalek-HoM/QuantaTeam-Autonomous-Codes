package org.firstinspires.ftc.teamcode.testingTrajectories.finalAutoCodes;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.classes.blobDetectionTest;
import org.firstinspires.ftc.teamcode.tuning.TuningOpModes;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

public class red1 extends LinearOpMode {

    blobDetectionTest detector = new blobDetectionTest(telemetry);
    OpenCvCamera webCam;

    int tpPos = blobDetectionTest.tp_zone;

    // slides
    DcMotor slideL;
    DcMotor slideR;

    // arm base core
    DcMotor armBase;

    // gripper arm core
    DcMotor gripperArm;

    // gripper tilting servo
    Servo tilting;

    // grippers
    Servo gripperL;
    Servo gripperR;

    @Override
    public void runOpMode() throws InterruptedException {

        // webCam and pipeline init
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "webCam1");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webCam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);

        webCam.setPipeline(detector);

        slideL = hardwareMap.get(DcMotor.class, "SlideL");
        slideR = hardwareMap.get(DcMotor.class, "SlideR");
        armBase = hardwareMap.get(DcMotor.class, "armBase");
        gripperArm = hardwareMap.get(DcMotor.class, "gripperArm");
        tilting = hardwareMap.get(Servo.class, "tilting");
        gripperL = hardwareMap.get(Servo.class, "gripperL");
        gripperR = hardwareMap.get(Servo.class, "gripperR");

        // pidf values
        ((DcMotorEx) armBase).setVelocityPIDFCoefficients(40, 0.1, 0.1, 50);
        ((DcMotorEx) armBase).setPositionPIDFCoefficients(5);
        ((DcMotorEx) gripperArm).setVelocityPIDFCoefficients(40, 0.1, 0.1, 50);
        ((DcMotorEx) gripperArm).setPositionPIDFCoefficients(5);

        // reversed motors
        gripperL.setDirection(Servo.Direction.REVERSE);
        gripperArm.setDirection(DcMotorSimple.Direction.REVERSE);


        // stop and reset encoder
        slideL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armBase.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        gripperArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        gripperR.setPosition(1);
        gripperL.setPosition(1);

        webCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened() {
                // Usually this is where you'll want to start streaming from the camera (see section 4) this is in the easy opencv docs
                webCam.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);


            }
            @Override
            public void onError(int errorCode) {
                /*
                 * This will be called if the camera could not be opened
                 */
                telemetry.addLine("the camera is dead! help!");
            }
        });



        waitForStart();

        telemetry.addLine("tpPos's indexes; 1=left, 2=mid, 3=right ");
        telemetry.addLine("");
        telemetry.addData("team prop's zone detected was ", tpPos);
        telemetry.update();

        if (tpPos == 1) {
            Pose2d beginPose = new Pose2d(11, -60, Math.PI / 2);
            if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
                MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);


                GripperArm(540, 0.2);
                sleep(1000);
                tilting.setPosition(0.8);



                Actions.runBlocking(
                        drive.actionBuilder(beginPose)

                                .lineToY(-50)
                                .turnTo(2.129302)
                                .waitSeconds(1)
                                .build());



                gripperR.setPosition(0);
                sleep(500);

                GripperArm(0, 0.2);
                ArmBase(230,0.5);
                gripperR.setPosition(1);

                sleep(1000);

                tilting.setPosition(0.7);

                Actions.runBlocking(
                        drive.actionBuilder(new Pose2d(11, -50, 2.129302))
                                .turnTo(1*Math.PI/2)
                                .lineToY(-51)

                                .turnTo(2 * Math.PI)
                                .lineToX(38)

                                .turnTo(Math.PI/2 )

                                .lineToY(-23)


                                .turnTo(2* Math.PI)


                                .build());



//            sleep(1000);
                Open_Tilting();
                GripperArm(450, 0.2);
                sleep(2000);

                Actions.runBlocking(
                        drive.actionBuilder(new Pose2d(38, -24, 2* Math.PI))
                                .lineToX(42)

                                .build());

                gripperL.setPosition(0);
                sleep(500);
                gripperL.setPosition(1);

                GripperArm(0, 0.2);
                sleep(500);

                ArmBase(0, 0.5);

                Actions.runBlocking(
                        drive.actionBuilder(new Pose2d(42, -24, 2 * Math.PI))
                                .lineToX(50)
                                .turnTo(3 * Math.PI/2)
                                .lineToY(-55)
                                .build());


            }


        } else if (tpPos == 2) {
            Pose2d beginPose = new Pose2d(11, -60, Math.PI / 2);
            if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
                MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);


                GripperArm(540, 0.2);
                sleep(1000);
                tilting.setPosition(0.8);



                Actions.runBlocking(
                        drive.actionBuilder(beginPose)

                                .lineToY(-45)
                                .waitSeconds(1)
                                .build());



                gripperR.setPosition(0);
                sleep(500);

                GripperArm(10, 0.2);
                ArmBase(230,0.5);
                gripperR.setPosition(1);

                sleep(1000);

                tilting.setPosition(0.7);

                Actions.runBlocking(
                        drive.actionBuilder(new Pose2d(11, -44, Math.PI / 2))

                                .lineToY(-51)

                                .turnTo(2 * Math.PI)
                                .lineToX(35)

                                .turnTo(Math.PI/2 )

                                .lineToY(-30)


                                .turnTo(2* Math.PI)


                                .build());



//            sleep(1000);
                Open_Tilting();
                GripperArm(450, 0.2);
                sleep(2000);

                Actions.runBlocking(
                        drive.actionBuilder(new Pose2d(35, -30, 2* Math.PI))

                                .lineToX(42)

                                .build());

                gripperL.setPosition(0);
                sleep(500);
                gripperL.setPosition(1);

                GripperArm(10, 0.2);
                sleep(500);

                ArmBase(0, 0.5);

                Actions.runBlocking(
                        drive.actionBuilder(new Pose2d(35, -30, 2 * Math.PI))


                                .lineToX(50)
                                .turnTo(3 * Math.PI/2)
                                .lineToY(-52)
                                .build());




            }

        } else {
            Pose2d beginPose = new Pose2d(11, -60, Math.PI / 2);
            if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
                MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);


                GripperArm(540, 0.2);
                sleep(1000);
                tilting.setPosition(0.8);



                Actions.runBlocking(
                        drive.actionBuilder(beginPose)

                                .lineToY(-50)
                                .turnTo(1.134464)
                                .waitSeconds(1)
                                .build());



                gripperR.setPosition(0);
                sleep(500);

                GripperArm(0, 0.2);
                ArmBase(230,0.5);
                gripperR.setPosition(1);

                sleep(1000);

                tilting.setPosition(0.7);

                Actions.runBlocking(
                        drive.actionBuilder(new Pose2d(11, -50, 1.134464))
                                .turnTo(1*Math.PI/2)
                                .lineToY(-51)

                                .turnTo(2 * Math.PI)
                                .lineToX(35)

                                .turnTo(Math.PI/2 )

                                .lineToY(-36)


                                .turnTo(2* Math.PI)


                                .build());



//            sleep(1000);
                Open_Tilting();
                GripperArm(450, 0.2);
                sleep(2000);

                Actions.runBlocking(
                        drive.actionBuilder(new Pose2d(35, -36, 2* Math.PI))
                                .lineToX(42)

                                .build());

                gripperL.setPosition(0);
                sleep(500);
                gripperL.setPosition(1);

                GripperArm(0, 0.2);
                sleep(500);

                ArmBase(0, 0.5);

                Actions.runBlocking(
                        drive.actionBuilder(new Pose2d(42, -36, 2 * Math.PI))
                                .lineToX(50)
                                .turnTo(3 * Math.PI/2)
                                .lineToY(-55)
                                .build());




            }
        }

        webCam.stopStreaming();




    }

    private void ArmBase(int Ticks, double Power) {
        armBase.setTargetPosition(Ticks);
        armBase.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armBase.setPower(Power);
    }


    private void GripperArm(int Ticks2, double Power2) {
        gripperArm.setTargetPosition(Ticks2);
        gripperArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        gripperArm.setPower(Power2);
    }

    private void Open_GripperL() {
        gripperR.setPosition(0);
        gripperL.setPosition(0);
        sleep(200);
    }


    private void Close_Tilting() {
        tilting.setPosition(0.4);
        sleep(200);
    }

    private void Open_Tilting() {
        tilting.setPosition(0.65);
        sleep(200);
    }


    private void Close_GripperL() {
        gripperR.setPosition(1);
        gripperL.setPosition(1);
        sleep(200);

    }

}

