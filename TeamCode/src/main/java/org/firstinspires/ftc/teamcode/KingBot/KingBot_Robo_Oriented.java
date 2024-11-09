package org.firstinspires.ftc.teamcode.KingBot;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

// Uses Field Centric Controlling
// The forward direction is defined as the FORWARD when the bot initializes
@TeleOp(name = "King Bot (Robo-Oriented)", group = "Linear OpMode")

public class KingBot_Robo_Oriented extends LinearOpMode {
    final double VERT_ENCODER_RESOLUTION = 537.7;
    final double VERT_GEAR_RADIUS = 3.82; // cm
    final double CM_TO_ENCODER_FACTOR = VERT_ENCODER_RESOLUTION/(2*Math.PI * VERT_GEAR_RADIUS); // cm * THIS = encoder position

    final double FULL_EXTENT_VERT_CM = 30;
    final int FULL_EXTENT_VERT_ENCODERS = (int) (FULL_EXTENT_VERT_CM * CM_TO_ENCODER_FACTOR);
    final int MIN_EXTENT_VERT_ENCODERS = 0;
    final double VERT_POWER = 0.5;

    final double FULL_EXTENT_HORI_CM = 65;
    final int FULL_EXTENT_HORI_ENCODERS = (int) (FULL_EXTENT_HORI_CM * CM_TO_ENCODER_FACTOR);
    final int MIN_EXTENT_HORI_ENCODERS = 0;
    final double HORI_POWER = 0.5;

    final double[] HORI_BOUNDS_CM = {0, FULL_EXTENT_HORI_CM};
    final int[] HORI_BOUNDS_ENCODERS = {(int) (HORI_BOUNDS_CM[0] * CM_TO_ENCODER_FACTOR), (int) (HORI_BOUNDS_CM[1] * CM_TO_ENCODER_FACTOR)};

    // Value Variables
    double flipperPower = 0.5;
    double intakePower = 1.0;

    // Declare our motors
    // Make sure your ID's match your configuration
    DcMotor frontLeftMotor;
    DcMotor backLeftMotor;
    DcMotor frontRightMotor;
    DcMotor backRightMotor;

    DcMotor vertSlide;
    DcMotor horiSlide;

    Servo intake;
    Servo flipper;

    IMU imu;

    @Override
    public void runOpMode() throws InterruptedException {
        frontLeftMotor = hardwareMap.dcMotor.get("left_front_drive");
        backLeftMotor = hardwareMap.dcMotor.get("left_back_drive");
        frontRightMotor = hardwareMap.dcMotor.get("right_front_drive");
        backRightMotor = hardwareMap.dcMotor.get("right_back_drive");

        vertSlide = hardwareMap.dcMotor.get("vert");
        horiSlide = hardwareMap.dcMotor.get("hori");

        intake = hardwareMap.servo.get("intake");
        flipper = hardwareMap.servo.get("flipper");

        imu = hardwareMap.get(IMU.class, "imu");

        // ENCODERS
        vertSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        horiSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Retrieve the IMU from the hardware map
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        waitForStart();

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double max;

            double axial = -gamepad1.left_stick_y;
            double lateral = gamepad1.left_stick_x;
            double yaw = gamepad1.right_stick_x;

            double leftFrontPower  = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower   = axial - lateral + yaw;
            double rightBackPower  = axial + lateral - yaw;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower  /= max;
                rightFrontPower /= max;
                leftBackPower   /= max;
                rightBackPower  /= max;
            }

            frontLeftMotor.setPower(leftFrontPower);
            frontRightMotor.setPower(rightFrontPower);
            backLeftMotor.setPower(leftBackPower);
            backRightMotor.setPower(rightBackPower);


            /**
             * MOTORS
             * **/
            double vert_power = gamepad2.right_stick_y;
            vertSlide.setPower(vert_power);

            double hori_power = gamepad2.left_stick_y;
            horiSlide.setPower(hori_power);

            /**
             * SERVOS
             * **/

            if (gamepad2.dpad_up) {
                intake.setPosition(1);
            }
            if (gamepad2.dpad_down) {
                intake.setPosition(0);
            }

            if (gamepad2.right_bumper) {
                flipper.setPosition(1);
            }
            if (gamepad2.left_bumper) {
                flipper.setPosition(0);
            }

            /**
             * ENCODERS
             * **/
            if (gamepad2.y) {
                runMotorToEncoderPosition(vertSlide, FULL_EXTENT_VERT_ENCODERS, VERT_POWER);
            }
            else if (gamepad2.b)
            {
                runMotorToEncoderPosition(vertSlide, MIN_EXTENT_VERT_ENCODERS, -VERT_POWER);
            }

            if (gamepad2.x) {
                runMotorToEncoderPosition(horiSlide, -FULL_EXTENT_HORI_ENCODERS, -HORI_POWER);
            }
            if (gamepad2.a)
            {
                runMotorToEncoderPosition(horiSlide, MIN_EXTENT_HORI_ENCODERS, HORI_POWER);
            }

            /** ENCODER STOPS FOR ECCENTRIC DESIGNS **/
            while (horiSlide.getCurrentPosition() > HORI_BOUNDS_ENCODERS[1]) {
                horiSlide.setPower(HORI_POWER);
            }
            horiSlide.setPower(hori_power);

            telemetry.update();

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
