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
@Autonomous(name = "copiaDeSeguridad")
public class copiaDeSeguridad extends LinearOpMode {

    private DcMotor leftRear;
    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor rightRear;
    private DcMotor LinearMotion;
    private DcMotor Codo;
    private DcMotor Muneca;
    private Servo Dedo1;
    private Servo Dedo2;
    private int ciclos_a_sec = 25000;

    public void hang_the_specimen(){
        Muneca.setTargetPosition(90);
        Muneca.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Codo.setTargetPosition(-1300);
        Codo.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Dedo1.setPosition(1); // dedo1 en 1 es cerrado
        Dedo2.setPosition(0); // dedo 2 en 0 es cerrado
        int ciclos_sec = 32767;

        for(int i = 0; i<ciclos_sec*3; i++){
            Codo.setPower(1); // SUBIR BRAZO
        }
        for(int i = 0; i<ciclos_sec*3; i++){
            Codo.setPower(1); // MANTENER BRAZO
            Muneca.setPower(1); // EXTENDER MUNECA
        }


        Codo.setTargetPosition(-850);
        Codo.setPower(-0.3); // BAJA EL BRAZO PARA COLGAR
        sleep(800);
        // abrir para soltar sample
        Dedo1.setPosition(0);
        Dedo2.setPosition(1);
        sleep(500);
        // cerrar para guardar garra
        Dedo1.setPosition(1);
        Dedo2.setPosition(0);
        sleep(200);
        for(int i = 0; i<ciclos_sec; i++){
            Muneca.setPower(0.7); // guarda muneca
        }

        Codo.setPower(0);
        Muneca.setTargetPosition(0);


    }

    private void frenar(){
        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);
    }


    public void runOpMode() throws InterruptedException {
        leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightRear = hardwareMap.get(DcMotor.class, "rightRear");

        leftRear.setDirection(DcMotor.Direction.REVERSE);
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setDirection(DcMotor.Direction.REVERSE);

        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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
                .lineTo(new Vector2d(40, 10))
                .build();

        TrajectorySequence ts2 = drive.trajectorySequenceBuilder(startPose)
                //Se posicion para ir a recoger especimen
                .strafeLeft(17)
                .build();

        TrajectorySequence ts3 = drive.trajectorySequenceBuilder(startPose)
                // Va hacia adelante para recoger especimen
                .forward(70)
                .build();




        waitForStart();
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        if (isStopRequested()) return;

        // Follow the trajectory.
        drive.followTrajectorySequence(ts);

        hang_the_specimen();
        sleep(600);



        // AQUI EMPIEZA ESTE SEGMENTO DE CODIGO A PULSOOO
        for(int i = 0; i<ciclos_a_sec*2.5; i++){ //DERECHA DETRAS DE ESPECIMENES (ESTAN INVERTIDOS LOS MOTORES)
            leftFront.setPower(-0.4);
            leftRear.setPower(0.4);
            rightFront.setPower(0.4);
            rightRear.setPower(-0.4);
        }
        frenar();
        sleep(100);

        for(int i = 0; i<ciclos_a_sec*0.2; i++){ // PRIMER ROTAR LEVEMETE HACIA LA IZQUIERDA SOBRE SU PROPIO EJE
            leftFront.setPower(-0.4);
            leftRear.setPower(-0.4);
            rightFront.setPower(0.4);
            rightRear.setPower(0.4);
        }
        frenar();
        sleep(100);

        for(int i = 0; i<ciclos_a_sec*0.9; i++){ //ADELANTE (ESTAN INVERTIDOS LOS MOTORES)
            leftFront.setPower(0.7);
            rightFront.setPower(0.7);
            leftRear.setPower(0.7);
            rightRear.setPower(0.7);
        }
        frenar();
        sleep(100);

        for(int i = 0; i<ciclos_a_sec*0.9; i++){ //DERECHA FRENTE ESPECIMEN(ESTAN INVERTIDOS LOS MOTORES)
            leftFront.setPower(-0.4);
            leftRear.setPower(0.4);
            rightFront.setPower(0.4);
            rightRear.setPower(-0.4);
        }
        frenar();
        sleep(100);

        for(int i = 0; i<ciclos_a_sec*2.5; i++){ //ATRAS Y ARRASTRA PRIMER ESPECIMEN (ESTAN INVERTIDOS LOS MOTORES)
            leftFront.setPower(-0.4);
            rightFront.setPower(-0.4);
            leftRear.setPower(-0.4);
            rightRear.setPower(-0.4);
        }
        frenar();
        sleep(100);

        for(int i = 0; i<ciclos_a_sec*0.1; i++){ //SEGUNDO ROTAR LEVEMETE HACIA LA IZQUIERDA SOBRE SU PROPIO EJE
            leftFront.setPower(-0.4);
            leftRear.setPower(-0.4);
            rightFront.setPower(0.4);
            rightRear.setPower(0.4);
        }
        frenar();
        sleep(100);

        for(int i = 0; i<ciclos_a_sec*2.3; i++){ //ADELANTE POR SEGUNDA VEZ (ESTAN INVERTIDOS LOS MOTORES)
            leftFront.setPower(0.4);
            rightFront.setPower(0.4);
            leftRear.setPower(0.4);
            rightRear.setPower(0.4);
        }
        frenar();
        sleep(100);

        for(int i = 0; i<ciclos_a_sec*0.5; i++){ //DERECHA FRENTE SEGUNDO ESPECIMEN(ESTAN INVERTIDOS LOS MOTORES)
            leftFront.setPower(-0.4);
            leftRear.setPower(0.4);
            rightFront.setPower(0.4);
            rightRear.setPower(-0.4);
        }
        frenar();
        sleep(100);


        for(int i = 0; i<ciclos_a_sec*2; i++){ //ATRAS Y ARRASTRA PRIMER ESPECIMEN (ESTAN INVERTIDOS LOS MOTORES)
            leftFront.setPower(-0.4);
            rightFront.setPower(-0.4);
            leftRear.setPower(-0.4);
            rightRear.setPower(-0.4);
        }
        frenar();
        sleep(100);


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