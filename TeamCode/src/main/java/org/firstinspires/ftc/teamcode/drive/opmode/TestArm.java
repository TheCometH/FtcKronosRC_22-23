package org.firstinspires.ftc.teamcode.drive.opmode;

import android.graphics.LinearGradient;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@TeleOp(name = "TestArm")
public class TestArm extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx expansion = hardwareMap.get(DcMotorEx.class, "expansion");
        expansion.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        expansion.setDirection(DcMotorSimple.Direction.FORWARD);
        expansion.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        expansion.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        waitForStart();
        while(opModeIsActive()){
            expansion.setPower(gamepad1.left_stick_y);
            telemetry.addData("position of the rotation encoder ", expansion.getCurrentPosition());
            telemetry.update();
        }
    }
}