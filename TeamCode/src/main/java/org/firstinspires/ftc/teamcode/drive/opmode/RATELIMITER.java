package org.firstinspires.ftc.teamcode.drive.opmode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import java.util.ArrayList;
import java.util.List;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "bob")
public class SmoothMovement extends LinearOpMode {
    private DcMotorEx leftFront, leftRear, rightRear, rightFront;
    private DcMotorEx expansion, traverse, rotation;
    private Servo tilt, clamp;

    Float prev_val_x = 0.0f;
    Float current_val_x;

    Float prev_val_y = 0.0f;
    Float current_val_y;

    final Float rate_limiter = 0.3f;

    private static double arm_ticks_per_rev = 5281.1 * 2;


    private Integer positionExpansion = 0;
    private Integer positionRotation = 0;
    private Integer positionTraverse = 0;

    private Float tiltPosition = 0f;

    private Float ughOffset =  -30f;
    private int irP;
    private int ieP;

    private int change;
    private int previous;
    private Boolean servoAutoThing = true;

    public Float help = 17130.2f;


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

//        rotation.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        rotation.setTargetPosition(rotation.getTargetPosition() - (int)(arm_ticks_per_rev/4));
//        rotation.setPower(1);
//
//        while (rotation.isBusy()){
//            rotation.setPower(1);
//        }
//
//        rotation.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        rotation.setPower(0);

//        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        expansion.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        rotation.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        traverse.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//


//        expansion.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        traverse.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

//        positionExpansion = expansion.getCurrentPosition();
//        positionRotation = rotation.getCurrentPosition();
//        positionTraverse = traverse.getCurrentPosition();


        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        expansion.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rotation.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        traverse.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        irP = rotation.getCurrentPosition();
        ieP = expansion.getCurrentPosition();
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
        Boolean isPivoting = false;
        Boolean isArm = false;


        /**
         *  Gamepad 1:
         *      Left stick;
         *          y: Chassis back and forth, x: Chassis left and right
         *      Bumper:
         *          Chassis turn left, right
         *  Gamepad 2:
         *      A: Close Claw
         *      B: Open Claw
         *      Left stick:
         *          x: Traverse left and right, y: Rotate right and left
         *      Right stick:
         *          x: Expand and Retract
         *      Bumper:0
         *          Left: tilt
         *
         * **/

        while(opModeIsActive()) {
            direction.clear();

            current_val_x = gamepad1.left_stick_x;

            current_val_y = gamepad1.left_stick_y;

            if (!rotation.isBusy()){
                rotation.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            }
            if (!expansion.isBusy()){
                expansion.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }

            if (gamepad2.dpad_down){
                help = help + 10;
            }
            if (gamepad2.dpad_up){
                help = help - 10;
            }
            if (gamepad2.y){
                servoAutoThing = false;
            }
            if (gamepad2.x){
                servoAutoThing = true;
            }

            if (servoAutoThing){
                godHelpMe();
            }

            // Forward
            if (gamepad1.left_stick_y > 0.3) {
                if (current_val_y > prev_val_y + rate_limiter) {
                    current_val_y = prev_val_y + rate_limiter;

                }
                direction.add("Forward");
                telemetry.addData("Forward, power", current_val_y);
                move(current_val_y);
            } else if (gamepad1.left_stick_y < -0.3) {
                if (current_val_y < prev_val_y - rate_limiter) {
                    current_val_y = prev_val_y - rate_limiter;

                }
                direction.add("Backward");
                telemetry.addData("Backward, power", current_val_y);

                move(current_val_y);
            } else if (gamepad1.right_stick_x < -0.3) {
                if (current_val_x < prev_val_x - rate_limiter) {
                    current_val_x = prev_val_x - rate_limiter;

                }
                direction.add("Right");
                telemetry.addData("Right, power", current_val_x);
                sides(-current_val_x);

            } else if (gamepad1.right_stick_x > 0.3) {
                if (current_val_x > prev_val_x + rate_limiter) {
                    current_val_x = prev_val_x + rate_limiter;
                }
                direction.add("Left");
                telemetry.addData("Left, power", gamepad1.right_stick_x);
                sides(-gamepad1.right_stick_x);
            }

            // Turn
            if(gamepad1.right_bumper) {

                direction.add("TURN");
                turning(-1);

            }

            // Turn
            else if(gamepad1.left_bumper) {
                direction.add("TURN");

                turning(1);
            }

            // Open Claw
            if(gamepad2.a) {
                openClaw();
            }

            // Close Claw
            if(gamepad2.b) {
                closeClaw();
                // unknown distance
            }

            // Traverse Right
            if(gamepad2.left_stick_x > 0.2) {
                telemetry.addData("Traverse right, power", gamepad2.left_stick_x);
                traversing(gamepad2.left_stick_x, 72);
            }

            // Traverse Left
            else if(gamepad2.left_stick_x < -0.2) {
                telemetry.addData("Traverse left, power", gamepad2.left_stick_x);
                traversing(gamepad2.left_stick_x, -72);
            }

            // Traverse Stop
            else {
                traverse.setPower(0);
            }

            telemetry.addData("triggerl", gamepad2.left_trigger);
            if (gamepad2.left_trigger>0 && !(rotation.isBusy() || expansion.isBusy())){
                goToPosition(931, 867);
            }
            else if (gamepad2.right_trigger > 0 && !(rotation.isBusy() || expansion.isBusy())){
                goToPosition(-1633, 1272);
            }

            // Expand Forward
            if (gamepad2.right_stick_y > 0.4) {
                telemetry.addData("Expand, power", gamepad2.right_stick_y);
                expand(-gamepad2.right_stick_y, -288);
            }

            // Expand Backward
            else if(gamepad2.right_stick_y < -0.4) {
                telemetry.addData("Expand, power", gamepad2.right_stick_y);
                expand(-gamepad2.right_stick_y , -288);
            }

            // Stop expand
            else {
                if (!expansion.isBusy()) {
                    expansion.setPower(0);
                }
            }

            // Rotate Right
            if(gamepad2.left_stick_y > 0.4) {
                telemetry.addData("Rotate right, power", gamepad2.left_stick_y);
                rotate((double)gamepad2.left_stick_y, 288);
            }

            // Rotate Left
            else if(gamepad2.left_stick_y < -0.4) {
                telemetry.addData("Rotate left, power", gamepad2.left_stick_y);
                rotate((double)gamepad2.left_stick_y, -288);
            }

            // Stop rotate
            else {
                if (!rotation.isBusy()) {
                    rotation.setPower(0);
                }
            }


            // Tilt Up or down idk
            if (gamepad2.left_bumper) {
                tiltNow(0.003f);
                telemetry.addData("Tilt", tilt.getPosition());
            }

            // Tilt Up or down idk
            if (gamepad2.right_bumper) {
                tiltNow(-0.003f);
                telemetry.addData("Tilt", tilt.getPosition());

            }

            if (direction.isEmpty()) {
                leftFront.setPower(0);
                leftRear.setPower(0);
                rightFront.setPower(0);
                rightRear.setPower(0);

            }

            telemetry.addData("tiltPosition: ", tilt.getPosition());
            telemetry.addData("Rotation Position", rotation.getCurrentPosition() - irP);
            telemetry.addData("Expansion Position", expansion.getCurrentPosition() - ieP);
            telemetry.addData("Traverse Position", traverse.getCurrentPosition() - ieP);
            telemetry.update();

            prev_val_y = current_val_y;
            prev_val_x = current_val_x;
        }
    }


    public void tiltNow(Float distance) {
        tiltPosition = Range.clip((float) tilt.getPosition() + (float) distance, 0, 1);
        tilt.setPosition(tiltPosition);
    }

    public void goToPosition(int rotationPos, int expansionPos) {
        rotation.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        expansion.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rotation.setTargetPosition(rotationPos + irP);
        expansion.setTargetPosition(expansionPos + ieP);

        rotation.setPower(1);
        expansion.setPower(1);

        while (rotation.isBusy() || expansion.isBusy()){

        }
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

    private float mtod(){

        return -((rotation.getCurrentPosition() - irP) * 360f / help);
    }



    private void godHelpMe(){

        tilt.setPosition((90f - mtod()) / 180f);


    }
    private void godHelpMe2(){
        tilt.setPosition((90f + mtod()) / 180f);

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
        clamp.setPosition(0);

    }

    public void openClaw() {
        clamp.setPosition(0.5);
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
