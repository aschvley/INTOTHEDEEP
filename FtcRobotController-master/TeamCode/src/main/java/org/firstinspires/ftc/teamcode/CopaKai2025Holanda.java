package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "CopaKai2025Holanda (Blocks to Java)")
public class CopaKai2025Holanda extends LinearOpMode {

    private DcMotor leftBack;
    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor rightBack;
    private DcMotor LinearMotion;
    private DcMotor Codo;
    private Servo ServoDer;
    private Servo ServoIzq;

    private void Motors_Setup() {
        // Drivetrain
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.FORWARD);

        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Linear, Arm 1st
        LinearMotion.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Codo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Linear
        LinearMotion.setDirection(DcMotor.Direction.REVERSE);
        LinearMotion.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LinearMotion.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Arm
        Codo.setDirection(DcMotor.Direction.REVERSE);
        Codo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Codo.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void runOpMode() {

    }
}