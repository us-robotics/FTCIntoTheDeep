package org.firstinspires.ftc.teamcode.KingBot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


// Uses Field Centric Controlling
// The forward direction is defined as the FORWARD when the bot initializes
@TeleOp(name = "King Bot (Field Centric)", group = "Linear OpMode")

public class KingBot extends LinearOpMode {
    final double VERT_ENCODER_RESOLUTION = 537.7;
    final double VERT_GEAR_RADIUS = 3.82; // cm
    final double CM_TO_ENCODER_FACTOR = VERT_ENCODER_RESOLUTION/(2*Math.PI * VERT_GEAR_RADIUS); // cm * THIS = encoder position

    final double FULL_EXTENT_VERT_CM = 30;
    final int FULL_EXTENT_VERT_ENCODERS = (int) (FULL_EXTENT_VERT_CM * CM_TO_ENCODER_FACTOR);
    final int MIN_EXTENT_VERT_ENCODERS = 0;
    final double VERT_POWER = 0.5;

    final int MIN_EXTENT_HORI_ENCODERS = (int) (12 * CM_TO_ENCODER_FACTOR);
    final double FULL_EXTENT_HORI_CM = 62;
    final int FULL_EXTENT_HORI_ENCODERS = (int) (FULL_EXTENT_HORI_CM * CM_TO_ENCODER_FACTOR);
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
    Servo swing;
    Servo roof;

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
        swing = hardwareMap.servo.get("swing");
        roof = hardwareMap.servo.get("roof");

        imu = hardwareMap.get(IMU.class, "imu");

        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        frontLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        backRightMotor.setDirection(DcMotor.Direction.REVERSE);

/*        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        backRightMotor.setDirection(DcMotor.Direction.FORWARD);*/

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

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            /**
             * MOVEMENT
             * **/
            double y = gamepad1.left_stick_y;
            double x = -gamepad1.left_stick_x;
            double rx = -gamepad1.right_stick_x;
            moveRobotAbsolute(x, y, rx);

            // This button choice was made so that it is hard to hit on accident,
            // it can be freely changed based on preference.
            // The equivalent button is start on Xbox-style controllers.
            if (gamepad1.options) {
                imu.resetYaw();
            }

            /**
             * MOTORS
             * **/
            double vert_power = gamepad2.right_stick_y;
            vertSlide.setPower(vert_power);


            /**
             * SERVOS
             * **/

            if (gamepad2.dpad_up) {
                intake.setPosition(1);
            }
            if (gamepad2.dpad_down) {
                intake.setPosition(0);
            }

            // Down
            if (gamepad2.dpad_left) {
                swing.setPosition(1);
            }
            // Up
            if (gamepad2.dpad_right) {
                swing.setPosition(0);
            }

            // Open
            if (gamepad1.left_bumper) {
                roof.setPosition(0);
            }
            // Close
            if (gamepad1.right_bumper) {
                roof.setPosition(1);
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

            /** ENCODER STOPS FOR ECCENTRIC DESIGNS **/
/*            while (horiSlide.getCurrentPosition() > HORI_BOUNDS_ENCODERS[1]) {
                horiSlide.setPower(HORI_POWER);
            }
            horiSlide.setPower(hori_power);*/

            telemetry.update();

            if (gamepad1.x) {
                scoreInitiate();
            }

            if (gamepad1.y) {
                scoreFinish();
            }

        }

    }
    /**
     * Move robot according to desired axes motions for field oriented
     * <p>
     * Positive X is forward
     * <p>
     * Positive Y is strafe left
     * <p>
     * Positive Yaw is counter-clockwise
     */
    public void moveRobotAbsolute(double x, double y, double rx) {
        // Calculate wheel powers.

        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // Rotate the movement direction counter to the bot's rotation
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        //rotX = rotX * 1.1;  // Counteract imperfect strafing

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        /**
         * SWEENEY MODIFICATION
         */
        frontLeftPower *= -1;
        frontRightPower *= -1;
        backLeftPower *= -1;
        backRightPower *= -1;

        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
        max = Math.max(max, Math.abs(backLeftPower));
        max = Math.max(max, Math.abs(backRightPower));

        if (max > 1.0) {
            frontLeftPower /= max;
            frontRightPower /= max;
            backLeftPower /= max;
            backRightPower /= max;
        }

        // Send powers to the wheels.
        frontLeftMotor.setPower(frontLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backLeftMotor.setPower(backLeftPower);
        backRightMotor.setPower(backRightPower);
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
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void scoreInitiate() {

        roof.setPosition(0.9);
        sleep(250);

        swing.setPosition(1.1);
        sleep(250);

        //runMotorToEncoderPosition(vertSlide, FULL_EXTENT_VERT_ENCODERS, VERT_POWER);

    }

    public void scoreFinish() {
/*        swing.setPosition(1);
        sleep(250);

        roof.setPosition(0.1);
        sleep(250);

        roof.setPosition(0.9);
        sleep(250);

        runMotorToEncoderPosition(vertSlide, MIN_EXTENT_VERT_ENCODERS, -VERT_POWER);*/
        sleep(250);

        swing.setPosition(0);
        sleep(2000);

        roof.setPosition(0);
        sleep(250);

        sleep(250);
        swing.setPosition(1);

    }

}
