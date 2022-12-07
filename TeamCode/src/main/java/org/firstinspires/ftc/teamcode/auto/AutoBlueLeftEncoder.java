package org.firstinspires.ftc.teamcode.auto;

import android.graphics.HardwareRenderer;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.auto.cameradetection.CameraDetectionV2;

@Autonomous(name = "AutoBlueLeftEncoder")
public class AutoBlueLeftEncoder extends LinearOpMode {

    DcMotorEx leftFront;
    DcMotorEx leftRear;
    DcMotorEx rightFront;
    DcMotorEx rightRear;

    static double wheelCircumference = 2 * 1.8898 * Math.PI;
    static double ticks_per_rev = 537.6;
    static double rotations;
    static int ticks;

    HardwareMap hw = null;

    @Override
    public void runOpMode() throws InterruptedException {
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");

        waitForStart();

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftRear.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightRear.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);

        CameraDetectionV2 camera = new CameraDetectionV2();
        camera.initTele(telemetry);
        camera.initCamera(hardwareMap);
        camera.detect();
        camera.update();

        //Initialize park to receive parking location
        int parkNumber = camera.check();

        straight(1.0, distance(12));
        sleep(3000);
        telemetry.clearAll();
        if(parkNumber == 1) {
            telemetry.addLine("1");
            telemetry.update();
        } else if (parkNumber == 2) {
            telemetry.addLine("2");
            telemetry.update();
        } else if (parkNumber == 3) {
            telemetry.addLine("3");
            telemetry.update();
        } else {
            telemetry.addLine("0");
            telemetry.update();
        }
        straight(1.0, distance(40));
    }

    public void straight(double power, int distance) {
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftRear.setTargetPosition(distance);
        leftFront.setTargetPosition(distance);
        rightRear.setTargetPosition(distance);
        rightFront.setTargetPosition(distance);

        leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftRear.setPower(power);
        leftFront.setPower(power);
        rightRear.setPower(power);
        rightFront.setPower(power);

        while (leftFront.isBusy() && leftRear.isBusy() && rightRear.isBusy() && rightFront.isBusy()) {

        }

        leftRear.setPower(0);
        leftFront.setPower(0);
        rightRear.setPower(0);
        rightFront.setPower(0);

        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addLine("Straight Path");
        telemetry.update();
    }

    public void strafe(double power, int distance) {
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftRear.setTargetPosition(distance);
        leftFront.setTargetPosition(distance);
        rightRear.setTargetPosition(distance);
        rightFront.setTargetPosition(distance);

        leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftRear.setPower(-power);
        leftFront.setPower(power);
        rightRear.setPower(power);
        rightFront.setPower(-power);

        while (leftFront.isBusy() && leftRear.isBusy() && rightRear.isBusy() && rightFront.isBusy()) {

        }

        leftRear.setPower(0);
        leftFront.setPower(0);
        rightRear.setPower(0);
        rightFront.setPower(0);

        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addLine("Straight Path");
        telemetry.update();
    }

    public void turn(double power, int distance) {
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftRear.setTargetPosition(distance);
        leftFront.setTargetPosition(distance);
        rightRear.setTargetPosition(distance);
        rightFront.setTargetPosition(distance);

        leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftRear.setPower(power);
        leftFront.setPower(power);
        rightRear.setPower(-power);
        rightFront.setPower(-power);

        while (leftFront.isBusy() && leftRear.isBusy() && rightRear.isBusy() && rightFront.isBusy()) {

        }

        leftRear.setPower(0);
        leftFront.setPower(0);
        rightRear.setPower(0);
        rightFront.setPower(0);

        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addLine("Straight Path");
        telemetry.update();
    }

    public int distance(double inches) {
        rotations = inches/wheelCircumference;
        ticks = (int) (rotations * ticks_per_rev);
        return ticks;
    }
}