package org.firstinspires.ftc.teamcode.TonyBot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


// Uses Field Centric Controlling
// The forward direction is defined as the FORWARD when the bot initializes
@TeleOp(name = "Tony Bot (Field Centric)", group = "Linear OpMode")
public class TonyBot extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        // Value Variables
        double tiltPower = 1.0;
        double slidePower = 1.0;

        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("left_front_drive");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("left_back_drive");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("right_front_drive");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("right_back_drive");

        DcMotor tilt = hardwareMap.dcMotor.get("tilt");
        DcMotor slide = hardwareMap.dcMotor.get("slide");
        Servo grab = hardwareMap.servo.get("grab");
        Servo wrist = hardwareMap.servo.get("wrist");

/*
        frontLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        backRightMotor.setDirection(DcMotor.Direction.REVERSE);
*/
        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        backRightMotor.setDirection(DcMotor.Direction.FORWARD);

        // Retrieve the IMU from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            /**
             * MOVEMENT
             * **/
            double x = gamepad1.left_stick_y;
            double y = -gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            // This button choice was made so that it is hard to hit on accident,
            // it can be freely changed based on preference.
            // The equivalent button is start on Xbox-style controllers.
            if (gamepad1.options) {
                imu.resetYaw();
            }

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);

            /**
             * MOTORS
             * **/
            double tilt_power = gamepad1.right_trigger - gamepad1.left_trigger;
            tilt.setPower(tilt_power * tiltPower);

            double slide_power = gamepad1.right_stick_y;
            slide.setPower(slide_power * slidePower);

            /**
             * SERVOS
             * **/
            if (gamepad1.left_bumper) {
                grab.setPosition(1.0);
            } else if (gamepad1.right_bumper) {
                grab.setPosition(0.0);
            }

            if (gamepad1.a) {
                wrist.setPosition(1.0);
            } else if (gamepad1.b) {
                wrist.setPosition(0.0);
            }



            if (gamepad2.a) {
                backLeftMotor.setPower(1);
            } else {
                backLeftMotor.setPower(0);
            }

            if (gamepad2.b) {
                frontLeftMotor.setPower(1);
            } else {
                frontLeftMotor.setPower(0);
            }

            if (gamepad2.y) {
                backRightMotor.setPower(1);
            } else {
                backRightMotor.setPower(0);
            }

            if (gamepad2.x) {
                frontRightMotor.setPower(1);
            } else {
                frontRightMotor.setPower(0);
            }

        }


    }

    public void runMotorToEncoderPosition(DcMotor motor, int position, double power) {
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setTargetPosition(position);
        motor.setPower(power);
        while (motor.isBusy() && opModeIsActive()) {
            telemetry.addData("Name: ", motor.getPortNumber());
            telemetry.addData("Pos: ", motor.getCurrentPosition());
            telemetry.update();
        }
        motor.setPower(0);
    }
}
