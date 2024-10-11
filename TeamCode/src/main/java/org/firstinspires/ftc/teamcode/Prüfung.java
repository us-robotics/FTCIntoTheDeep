package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


// Uses Field Centric Controlling
// The forward direction is defined as the FORWARD when the bot initializes
@TeleOp(name = "Prüfung (Test)", group = "Linear OpMode")
public class Prüfung extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        final double VERT_ENCODER_RESOLUTION = 384.5;
        final double VERT_GEAR_RADIUS = 3.82; // cm
        final double CM_TO_ENCODER_FACTOR = VERT_ENCODER_RESOLUTION/(2*Math.PI * VERT_GEAR_RADIUS); // cm * THIS = encoder position
        final double FULL_EXTENT_VERT_CM = 100;
        final int FULL_EXTENT_VERT_ENCODERS = (int) (FULL_EXTENT_VERT_CM * CM_TO_ENCODER_FACTOR);
        final int MIN_EXTENT_VERT_ENCODERS = 0;
        final double VERT_POWER = 0.3;

        // Value Variables

        DcMotor vertSlide = hardwareMap.dcMotor.get("test");
        vertSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        vertSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        vertSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            vertSlide.setPower(gamepad1.left_stick_y);

            if (gamepad1.y) {
                runMotorToEncoderPosition(vertSlide, -FULL_EXTENT_VERT_ENCODERS, -VERT_POWER);
            }
            if (gamepad1.x)
            {
                runMotorToEncoderPosition(vertSlide, MIN_EXTENT_VERT_ENCODERS, -VERT_POWER);
            }

            telemetry.addData("Pos", vertSlide.getCurrentPosition());
            telemetry.update();

        }
    }

    public void runMotorToEncoderPosition(DcMotor motor, int position, double power) {
        motor.setTargetPosition(position);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(power);
        while (motor.isBusy() && opModeIsActive()) {
            if (gamepad1.a) {
                break;
            }
            telemetry.addData("Target Pos", position);
            telemetry.addData("Current Pos", motor.getCurrentPosition());
            telemetry.update();
        }
        motor.setPower(0);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
