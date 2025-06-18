package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive; // Import your drive class
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

//RESPALDO
@Autonomous(name = "ROJO_ESTACIONARSE")
public class ROJO_ESTACIONARSE extends LinearOpMode {

    private DcMotor leftRear;
    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor rightRear;
    private DcMotor LinearMotion;
    private DcMotor Codo;
    private DcMotor Muneca;
    private Servo Dedo1;
    private Servo Dedo2;
    private DcMotor Codo2;
    private int ciclos_a_sec = 32767;
    int ultimaPosicionMuneca;

    public void pickup_specimen(){
        Muneca.setTargetPosition(120);
        Muneca.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Muneca.setPower(1);
        Dedo1.setPosition(1);
        Dedo2.setPosition(1);
        Codo.setTargetPosition(-269);
        Codo.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Codo.setPower(1);
        Codo2.setTargetPosition(-20);
        Codo2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Codo2.setPower(1);
        Dedo1.setPosition(0);
        Dedo2.setPosition(0);
        Muneca.setTargetPosition(0);
        Muneca.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Muneca.setPower(1);
        Codo.setTargetPosition(0);
        Codo.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Codo.setPower(1);
        Codo2.setTargetPosition(0);
        Codo2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Codo2.setPower(1);
        return;
    }

    public void hang_the_specimen(){
        Muneca.setTargetPosition(120);
        Muneca.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Codo.setTargetPosition(-880);
        Codo.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Dedo1.setPosition(1);
        Dedo2.setPosition(0);

        for(int i = 0; i<ciclos_a_sec*3; i++){
            Codo.setPower(1);
            Muneca.setPower(0.7);
        }

        sleep(500);

        for(int i = 0; i<ciclos_a_sec; i++){
            Muneca.setPower(0.7);
            Dedo1.setPosition(0);
            Dedo2.setPosition(1);
        }

        Codo.setPower(0);
        Muneca.setTargetPosition(0);
    }


    public void runOpMode() throws InterruptedException {

        //initialize arm motors
        LinearMotion = hardwareMap.get(DcMotor.class, "LinearMotion");
        Codo = hardwareMap.get(DcMotor.class, "Codo");
        Muneca = hardwareMap.get(DcMotor.class, "Muneca");
        Dedo1 = hardwareMap.get(Servo.class, "Dedo1");
        Dedo2 = hardwareMap.get(Servo.class, "Dedo2");

        //ITALY
        Codo.setDirection(DcMotor.Direction.REVERSE);
        LinearMotion.setDirection(DcMotor.Direction.REVERSE);
        Codo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LinearMotion.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Codo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LinearMotion.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Codo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LinearMotion.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Muneca.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Muneca.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //NOW
        /*
        //for starters
        LinearMotion.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Codo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Muneca.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // Linear
        LinearMotion.setDirection(DcMotor.Direction.FORWARD);
        LinearMotion.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LinearMotion.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // Arm
        Codo.setDirection(DcMotor.Direction.REVERSE);
        Codo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Codo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // Muneca
        Muneca.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Muneca.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);*/


        // Initialize Road Runner drive system (SampleMecanumDrive)
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // Set the starting pose.  IMPORTANT: Use inches and radians!
        Pose2d startPose = new Pose2d(26, -63.5, Math.toRadians(120)); //Starting pose
        drive.setPoseEstimate(startPose); // Tell Road Runner where the robot starts

        TrajectorySequence ts = drive.trajectorySequenceBuilder(startPose)
                //va a colgar specimen precargado
                .lineTo(new Vector2d(75, 14))
                .build();

        TrajectorySequence ts2 = drive.trajectorySequenceBuilder(startPose)
                //se posiciona para girar a la derecha
                .lineTo(new Vector2d(40, -40))
                .build();

        TrajectorySequence ts3 = drive.trajectorySequenceBuilder(startPose)
                .forward(120)
                .build();

        TrajectorySequence ts4 = drive.trajectorySequenceBuilder(startPose)
                .strafeLeft(40)
                .build();


        waitForStart();
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        if (isStopRequested()) return;

        // Follow the trajectory.
        drive.followTrajectorySequence(ts);

        hang_the_specimen();
        sleep(1500);
        Dedo1.setPosition(0.77);
        Dedo2.setPosition(-0.77);
        Muneca.setTargetPosition(-150);
        Muneca.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Muneca.setPower(0.7);
        telemetry.addData("heading", Muneca.getCurrentPosition());
        telemetry.update();
        while (Muneca.isBusy()) {
            idle();
        }
        telemetry.addData("heading", Muneca.getCurrentPosition());
        telemetry.update();
        Muneca.setPower(0);

        drive.followTrajectorySequence(ts2);

        drive.followTrajectorySequence(ts3);

        drive.followTrajectorySequence(ts4);
        sleep(2500);




        // Keep the OpMode alive while following.  The `isBusy()` check
        // is important to allow the control loop to run.
        while (opModeIsActive() && drive.isBusy()) {
            drive.update(); // Crucial: Update Road Runner's pose estimate

            // Add telemetry for debugging (optional, but highly recommended)
            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();
        }

    }
}