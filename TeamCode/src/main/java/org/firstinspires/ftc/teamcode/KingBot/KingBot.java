package org.firstinspires.ftc.teamcode.KingBot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


// Uses Field Centric Controlling
// The forward direction is defined as the FORWARD when the bot initializes
@TeleOp(name = "King Bot (Field Centric)", group = "Linear OpMode")
public class KingBot extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        final double VERT_ENCODER_RESOLUTION = 537.7;
        final double VERT_GEAR_RADIUS = 3.82; // cm
        final double CM_TO_ENCODER_FACTOR = VERT_ENCODER_RESOLUTION/(2*Math.PI * VERT_GEAR_RADIUS); // cm * THIS = encoder position

        final double FULL_EXTENT_VERT_CM = 30;
        final int FULL_EXTENT_VERT_ENCODERS = (int) (FULL_EXTENT_VERT_CM * CM_TO_ENCODER_FACTOR);
        final int MIN_EXTENT_VERT_ENCODERS = 0;
        final double VERT_POWER = 0.5;

        final double FULL_EXTENT_HORI_CM = 25;
        final int FULL_EXTENT_HORI_ENCODERS = (int) (FULL_EXTENT_HORI_CM * CM_TO_ENCODER_FACTOR);
        final int MIN_EXTENT_HORI_ENCODERS = 0;
        final double HORI_POWER = 0.5;

        // Value Variables
        double flipperPower = 0.5;
        double intakePower = 1.0;

        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("left_front_drive");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("left_back_drive");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("right_front_drive");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("right_back_drive");
        DcMotor flipper = hardwareMap.dcMotor.get("flipper");
        DcMotor intake = hardwareMap.dcMotor.get("intake");
        DcMotor vertSlide = hardwareMap.dcMotor.get("vert");
        DcMotor horiSlide = hardwareMap.dcMotor.get("hori");

        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
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

        // ENCODERS
        vertSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        horiSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

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
            double vert_power = gamepad2.right_stick_y;
            vertSlide.setPower(vert_power);

            double hori_power = gamepad2.left_stick_x;
            horiSlide.setPower(hori_power);

            intake.setPower(gamepad2.right_trigger - gamepad2.left_trigger);

            /**
             * SERVOS
             * **/
            if (gamepad2.left_bumper) {
                flipper.setPower(flipperPower);
            } else if (gamepad2.right_bumper) {
                flipper.setPower(-flipperPower);
            } else {
                flipper.setPower(0);
            }

            /**
             * ENCODERS
             * **/
/*            if (gamepad2.y) {
                runMotorToEncoderPosition(vertSlide, FULL_EXTENT_VERT_ENCODERS, VERT_POWER);
            }
            else if (gamepad2.b)
            {
                runMotorToEncoderPosition(vertSlide, MIN_EXTENT_VERT_ENCODERS, -VERT_POWER);
            }*/

            if (gamepad2.x) {
                runMotorToEncoderPosition(horiSlide, -FULL_EXTENT_HORI_ENCODERS, -HORI_POWER);
            }
            if (gamepad2.a)
            {
                runMotorToEncoderPosition(horiSlide, MIN_EXTENT_HORI_ENCODERS, HORI_POWER);
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
