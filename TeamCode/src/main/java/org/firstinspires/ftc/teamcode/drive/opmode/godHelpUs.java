package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="poqiwejasdfa")
public class godHelpUs extends LinearOpMode {

    private DcMotorEx leftFront, leftRear, rightRear, rightFront;
    private DcMotorEx expansion, traverse, rotation;
    private Servo tilt, clamp;

    HardwareMap hwMap = null;

    public void init(HardwareMap hardwaremap){
            long setTime = System.currentTimeMillis();

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

            leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
            leftRear.setDirection(DcMotorSimple.Direction.FORWARD);
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

            leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            expansion.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rotation.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            traverse.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void runOpMode(){
        init(hardwareMap);

        waitForStart();

        while(opModeIsActive()) {
            if (gamepad1.a) {
                telemetry.addData("left front", "forward");
                leftFront.setPower(1);
            }
            else if (gamepad1.a == false) {
                telemetry.addData("left front", "stop");
                leftFront.setPower(0);
            }
            if (gamepad1.b){
                telemetry.addData("right front", "forward");
                rightFront.setPower(1);
            }
            else if (gamepad1.b == false) {
                telemetry.addData("right front", "stop");
                rightFront.setPower(0);
            }
            if (gamepad1.x) {
                telemetry.addData("left back", "forward");
                leftRear.setPower(1);
            }
            else if (gamepad1.x == false) {
                telemetry.addData("left back", "stop");
                leftRear.setPower(0);
            }

            if (gamepad1.y) {
                telemetry.addData("right back", "forward");
                rightRear.setPower(1);
            }
            else if (gamepad1.y == false) {
                telemetry.addData("right back", "stop");
                rightRear.setPower(0);
            }
        }


    }
    public void tiltNow(double distance) {
        tilt.setPosition(distance);
    }

    public void closeClaw() {
        clamp.setPosition(0);

    }

    public void rotate(double power){
        rotation.setPower(power);
    }

    public void expand(double power){
        expansion.setPower(power);
    }

    public void openClaw() {
        clamp.setPosition(30);
    }

    public void traversing(double power) {
        traverse.setPower(power);
    }

    public void move(double power){
        rightFront.setPower(power);
        rightRear.setPower(power);
        leftRear.setPower(power);
        leftFront.setPower(power);
    }

    public void sides(double power){
        rightFront.setPower(-power);
        leftRear.setPower(power);
        rightRear.setPower(power);
        leftFront.setPower(-power);
    }

    public void turning(double power){
        rightFront.setPower(power);
        rightRear.setPower(power);
        leftFront.setPower(-power);
        leftRear.setPower(-power);
    }
}