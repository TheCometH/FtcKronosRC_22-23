package org.firstinspires.ftc.teamcode.drive.opmode;


import com.acmerobotics.roadrunner.drive.Drive;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import java.util.ArrayList;
import java.util.List;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.opencv.core.Mat;

import java.util.Arrays;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "polkioplkioplk")
public class TeleOp extends LinearOpMode {
    private DcMotorEx leftFront, leftRear, rightRear, rightFront;
    private DcMotorEx expansion, traverse, rotation;
    private Servo tilt, clamp;



    private Integer positionExpansion = 0;
    private Integer positionRotation = 0;
    private Integer positionTraverse = 0;

    private Float tiltPosition = 0f;
    boolean toggleparallel = true;



    HardwareMap hwMap = null;

    public void init(HardwareMap hardwareMap) {
        telemetry.addLine("init starting");
        telemetry.update();
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");

        expansion = hardwareMap.get(DcMotorEx.class, "expansion");
        traverse = hardwareMap.get(DcMotorEx.class, "traverse");
        rotation = hardwareMap.get(DcMotorEx.class, "rotation");
        tilt = hardwareMap.get(Servo.class, "tilt");
        clamp = hardwareMap.get(Servo.class, "clamp");

        tilt.setPosition(1);
        clamp.setPosition(0);



        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightRear.setDirection(DcMotorSimple.Direction.FORWARD);

        expansion.setDirection(DcMotorSimple.Direction.FORWARD);
        rotation.setDirection(DcMotorSimple.Direction.FORWARD);
        traverse.setDirection(DcMotorSimple.Direction.FORWARD);
        tilt.setDirection(Servo.Direction.FORWARD);
        clamp.setDirection(Servo.Direction.FORWARD);

        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);
        expansion.setPower(0);
        rotation.setPower(0);
        traverse.setPower(0);

//        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        expansion.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        rotation.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        traverse.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
        rotation.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        expansion.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        traverse.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

//        positionExpansion = expansion.getCurrentPosition();
//        positionRotation = rotation.getCurrentPosition();
//        positionTraverse = traverse.getCurrentPosition();


        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        expansion.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rotation.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        traverse.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    @Override
    public void runOpMode() {

        init(hardwareMap);
        telemetry.addLine("init done");
        telemetry.update();

        waitForStart();
        telemetry.addLine("starting now");
        telemetry.update();

        List<String> direction = new ArrayList<String>();


        while(opModeIsActive()) {

            if(gamepad1.a){
                //switch bool vlaue of toggle value
                if(toggleparallel){
                    toggleparallel = false;
                } else{
                    toggleparallel = true;
                }
            }

            if(toggleparallel) {
                if (rotation.getCurrentPosition() <= 600) {
                    tilt.setPosition(1 - (rotation.getCurrentPosition() * 5.0 / 6.0) / 600.0);
                }
            }

            if (gamepad1.left_stick_y > 0.4) {
                direction.add("forward");
                move(-1);
            }

            if (gamepad1.left_stick_y < -0.4) {
                direction.add("backward");
                move(1);
            }


            if (gamepad1.left_stick_x > 0.4) {
                direction.add("right");
                sides(1);
            }if (gamepad1.left_stick_x < -0.4) {
                direction.add("left");
                sides(-1);
            }


            if(gamepad1.right_bumper) {
                direction.add("TURN");

                turning(1);
            }
            else if(gamepad1.left_bumper) {
                direction.add("TURN");

                turning(-1);
            }


            if(gamepad2.a) {
                openClaw();
            }
            if(gamepad2.b) {
                closeClaw();
                // unknown distance
            }


            if(gamepad2.left_stick_x > 0.4) {
                traversing(1, 72);
            }


            if(gamepad2.left_stick_x < -0.4) {
                traversing(-1, -72);
            }


            if (gamepad2.right_stick_x > 0.4) {
                direction.add("expand");
                expand(-0.75, 288);
            }
            if(gamepad2.right_stick_x < -0.4) {
                direction.add("expand");
                expand(0.75, -288);
            }
            if(gamepad2.left_stick_y > 0.4) {
                direction.add("rotate");
                rotate(0.75, 288);
            }
            if(gamepad2.left_stick_y < -0.4) {
                direction.add("rotate");
                rotate(-0.75, -288);
            }

            //should only run if its not toggled
            if(!toggleparallel) {
                if (gamepad2.left_bumper) {
                    tiltNow(0.001f);
                    telemetry.addData("tilt", tilt.getPosition());
                }
                if (gamepad2.right_bumper) {
                    tiltNow(-0.001f);
                    telemetry.addData("tilt", tilt.getPosition());

                }
            }
            if (direction.isEmpty() ) {
                leftFront.setPower(0);
                leftRear.setPower(0);
                rightFront.setPower(0);
                rightRear.setPower(0);
                expansion.setPower(0);
                rotation.setPower(0);
                traverse.setPower(0);
                telemetry.addLine("stopping");
                telemetry.update();
            }

            telemetry.addData("direction", direction);
            telemetry.addData("tiltPosition: ", tiltPosition);
            telemetry.update();
        }
    }


    public void tiltNow(Float distance) {
        tiltPosition = tiltPosition + distance;
        telemetry.addData("tiltPosition: ", tiltPosition);
        telemetry.update();
        tilt.setPosition(tiltPosition);
    }



    public void rotate(Double power, int targetPosition) {
//        if (positionRotation < 1170 && targetPosition > 0) {
//            rotation.setTargetPosition(targetPosition + positionRotation);
//            rotation.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            rotation.setPower(power);
//            positionRotation = rotation.getCurrentPosition();
//        } else if (positionRotation >= 1170 && targetPosition < 0){
//            rotation.setTargetPosition(targetPosition + positionRotation);
//            rotation.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            rotation.setPower(power);
//            positionRotation = rotation.getCurrentPosition();
//        }
//        rotation.setTargetPosition(targetPosition + positionRotation);
//            rotation.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rotation.setPower(power);
//            positionRotation = rotation.getCurrentPosition();
    }

    public void expand(double power, int targetPosition){
//        if (positionExpansion < 6120 && targetPosition > -1) {
//            expansion.setTargetPosition(targetPosition);
//            expansion.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            expansion.setPower(power);
//            positionExpansion = expansion.getCurrentPosition();
//        }
//        else if (positionExpansion >= 6120 && targetPosition < 0){
//
//            expansion.setTargetPosition(targetPosition);
//            expansion.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            expansion.setPower(power);
//            positionExpansion = expansion.getCurrentPosition();
//        }
//        expansion.setTargetPosition(targetPosition);
//            expansion.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        expansion.setPower(power);
//            positionExpansion = expansion.getCurrentPosition();


    }
    public void traversing(double power, int targetPosition) {
//        if (positionTraverse < 1440 && positionTraverse > -1440) {
//            traverse.setTargetPosition(targetPosition);
//            traverse.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            traverse.setPower(power);
//            positionTraverse = traverse.getCurrentPosition();
//        }
//        else if (positionExpansion >= 1440 && targetPosition < 0){
//            traverse.setTargetPosition(targetPosition);
//            traverse.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            traverse.setPower(power);
//            positionTraverse = traverse.getCurrentPosition();
//        }
//        else if (positionExpansion <= -1440 && targetPosition > 0){
//            traverse.setTargetPosition(targetPosition);
//            traverse.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            traverse.setPower(power);
//            positionTraverse = traverse.getCurrentPosition();
//        }
//        traverse.setTargetPosition(targetPosition);
//        traverse.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        traverse.setPower(power);
//        positionTraverse = traverse.getCurrentPosition();
    }
    public void closeClaw() {
        clamp.setPosition(0.3);

    }

    public void openClaw() {
        clamp.setPosition(0.6);
    }



    public void move(double power){
        rightFront.setPower(power);
        rightRear.setPower(power);
        leftRear.setPower(power);
        leftFront.setPower(power);
    }

    public void sides(double power){
        rightFront.setPower(-power);
        leftRear.setPower(-power);
        rightRear.setPower(power);
        leftFront.setPower(power);
    }

    public void turning(double power){
        rightFront.setPower(power);
        rightRear.setPower(power);
        leftFront.setPower(-power);
        leftRear.setPower(-power);
    }
}