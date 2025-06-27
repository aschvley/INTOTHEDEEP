package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

//RESPALDO
@Autonomous(name = "ROJO_ESTACIONARSE")
public class ROJO_ESTACIONARSE extends LinearOpMode {

    private DcMotor LinearMotion;
    private DcMotor Codo;
    private DcMotor Muneca;
    private Servo Dedo1;
    private Servo Dedo2;
    private int ciclos_a_sec = 32767;
    int ultimaPosicionMuneca;

    public void pickup_specimen(){
        Muneca.setTargetPosition(105);
        Muneca.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Muneca.setPower(1);
        Dedo1.setPosition(1);
        Dedo2.setPosition(1);
        Codo.setTargetPosition(-269);
        Codo.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Codo.setPower(1);
        Dedo1.setPosition(0);
        Dedo2.setPosition(0);
        Muneca.setTargetPosition(0);
        Muneca.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Muneca.setPower(1);
        Codo.setTargetPosition(0);
        Codo.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Codo.setPower(1);
        return;
    }

    public void hang_the_specimen(){
        Muneca.setTargetPosition(90);
        Muneca.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Codo.setTargetPosition(-1300);
        Codo.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Dedo1.setPosition(1); // dedo1 en 1 es cerrado
        Dedo2.setPosition(0); // dedo 2 en 0 es cerrado

        for(int i = 0; i<ciclos_a_sec*3; i++){
            Codo.setPower(1); // SUBIR BRAZO
        }
        for(int i = 0; i<ciclos_a_sec*3; i++){
            Codo.setPower(1); // MANTENER BRAZO
            Muneca.setPower(1); // EXTENDER MUNECA
        }


        Codo.setTargetPosition(-1000);
        Codo.setPower(-0.3); // BAJA EL BRAZO PARA COLGAR
        sleep(500);
        // abrir para soltar sample
        Dedo1.setPosition(0);
        Dedo2.setPosition(1);
        sleep(500);
        Dedo1.setPosition(1);
        Dedo2.setPosition(0);
        sleep(500);
        for(int i = 0; i<ciclos_a_sec; i++){
            Muneca.setPower(0.7);
        }

        Codo.setPower(0);
        Muneca.setTargetPosition(0);


    }

    public void lo_q_hizo_carlos(){
        Dedo1.setPosition(0.77);
        Dedo2.setPosition(-0.77);
        Muneca.setTargetPosition(-150);
        Muneca.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Muneca.setPower(0.7);
        telemetry.addData("Muneca", Muneca.getCurrentPosition());
        telemetry.update();
        while (Muneca.isBusy()) {
            idle();
        }
        telemetry.addData("Muneca", Muneca.getCurrentPosition());
        telemetry.update();
        Muneca.setPower(0);
    }


    public void runOpMode() throws InterruptedException {

        //initialize arm motors
        LinearMotion = hardwareMap.get(DcMotor.class, "LinearMotion");
        Codo = hardwareMap.get(DcMotor.class, "Codo");
        Muneca = hardwareMap.get(DcMotor.class, "Muneca");
        Dedo1 = hardwareMap.get(Servo.class, "Dedo1");
        Dedo2 = hardwareMap.get(Servo.class, "Dedo2");

        //ITALY
        Codo.setDirection(DcMotor.Direction.FORWARD);
        LinearMotion.setDirection(DcMotor.Direction.REVERSE);
        Codo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LinearMotion.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Codo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LinearMotion.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Codo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LinearMotion.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Muneca.setDirection(DcMotor.Direction.REVERSE);
        Muneca.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Muneca.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Initialize Road Runner drive system (SampleMecanumDrive)
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // Set the starting pose.  IMPORTANT: Use inches and radians!
        Pose2d startPose = new Pose2d(26, -63.5, Math.toRadians(120)); //Starting pose
        drive.setPoseEstimate(startPose); // Tell Road Runner where the robot starts

        TrajectorySequence ts = drive.trajectorySequenceBuilder(startPose)
                //va a colgar specimen precargado
                .lineTo(new Vector2d(40, 11))
                .build();

        TrajectorySequence ts2 = drive.trajectorySequenceBuilder(startPose)
                //Se posicion para ir a recoger especimen
                .strafeLeft(40)
                .build();

        TrajectorySequence ts3 = drive.trajectorySequenceBuilder(startPose)
                // Va hacia adelante para recoger especimen
                .forward(55)
                .build();

        TrajectorySequence ts4 = drive.trajectorySequenceBuilder(startPose)
                // Va hacia la izquierda para recoger especimen
                .strafeLeft(20)
                .build();


        waitForStart();
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        if (isStopRequested()) return;

        // Follow the trajectory.
        drive.followTrajectorySequence(ts);

        hang_the_specimen();
        lo_q_hizo_carlos();

        drive.followTrajectorySequence(ts2);

        drive.followTrajectorySequence(ts3);

        drive.followTrajectorySequence(ts4);
        sleep(2500);

        sleep(1500);

        drive.followTrajectorySequence(ts2);



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