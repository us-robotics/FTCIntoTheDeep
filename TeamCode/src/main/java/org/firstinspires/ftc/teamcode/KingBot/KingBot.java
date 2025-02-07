package org.firstinspires.ftc.teamcode.KingBot;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


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

    final int MIN_EXTENT_HORI_ENCODERS = 0;
    final double FULL_EXTENT_HORI_CM = 80;
    final int FULL_EXTENT_HORI_ENCODERS = (int) (FULL_EXTENT_HORI_CM * CM_TO_ENCODER_FACTOR);
    final double HORI_POWER = 0.5;

    final double[] HORI_BOUNDS_CM = {0, FULL_EXTENT_HORI_CM};
    final int[] HORI_BOUNDS_ENCODERS = {(int) (HORI_BOUNDS_CM[0] * CM_TO_ENCODER_FACTOR), (int) (HORI_BOUNDS_CM[1] * CM_TO_ENCODER_FACTOR)};

    // Value Variables
    double flipperPower = 0.5;
    double intakePower = 1.0;

    double intakeFlipperMidPos = 0.75;
    double intakeFlipperMacroPos = 0.3;
    final double horiMacroPower = 0.75;

    float modifier = 1.0f;

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

    Servo intakeFlipper;
    // Servo intakeGrabber;
    CRServo intakeGrabber;
    float grabberPower = 0.7f;

    IMU imu;
    private SparkFunOTOS myOtos = null;

    int lastHoriPosition = 0;

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

        intakeFlipper = hardwareMap.servo.get("intakeFlipper");
        //intakeGrabber = hardwareMap.servo.get("intakeGrabber");
        intakeGrabber = hardwareMap.crservo.get("intakeGrabber");

        imu = hardwareMap.get(IMU.class, "imu");

        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        frontLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        backRightMotor.setDirection(DcMotor.Direction.REVERSE);

        vertSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

/*        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        backRightMotor.setDirection(DcMotor.Direction.FORWARD);*/

        // ENCODERS
        vertSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        vertSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        horiSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horiSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Retrieve the IMU from the hardware map
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        myOtos = hardwareMap.get(SparkFunOTOS.class, "sensor_otos");
        configureOtos();

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            /**
             * MOVEMENT
             * **/
            double y = gamepad1.left_stick_y;
            double x = -gamepad1.left_stick_x;
            double rx = -gamepad1.right_stick_x;
            moveRobotAbsolute(x * modifier, y * modifier, rx * modifier);

            // This button choice was made so that it is hard to hit on accident,
            // it can be freely changed based on preference.
            // The equivalent button is start on Xbox-style controllers.
            if (gamepad1.options) {
                imu.resetYaw();
            }

            if (gamepad1.b) {
                modifier = 0.5f;
            }

            if (gamepad1.a) {
                modifier = 1.0f;
            }

            /**
             * MOTORS
             * **/
            double vert_power = gamepad2.right_stick_y;
            vertSlide.setPower(vert_power);

            double hori_power = gamepad2.left_stick_x;

            if (hori_power > 0 && horiSlide.getCurrentPosition() > FULL_EXTENT_HORI_ENCODERS) {
                hori_power = 0;
            }

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

            // Up
            if (gamepad2.dpad_left) {
                swing.setPosition(1);
            }
            // Down
            if (gamepad2.dpad_right) {
                swing.setPosition(0);
            }

            if (gamepad2.left_bumper) {
                //intakeGrabber.setPosition(0);
                intakeGrabber.setPower(-grabberPower);
            } else if (gamepad2.right_bumper) {
                //intakeGrabber.setPosition(1);
                intakeGrabber.setPower(grabberPower);
            } else {
                intakeGrabber.setPower(0);
            }

            // Extendo Arm's Hand
            if (gamepad2.y) {
                intakeFlipper.setPosition(1);
            }
            if (gamepad2.b) {
                intakeFlipper.setPosition(intakeFlipperMidPos);
            }
            if (gamepad2.a) {
                intakeFlipper.setPosition(0);
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
            }
*/
/*            if (gamepad1.a) {
                runMotorToEncoderPosition(horiSlide, MIN_EXTENT_HORI_ENCODERS, HORI_POWER);
            }
            if (gamepad1.b) {
                runMotorToEncoderPosition(horiSlide, -FULL_EXTENT_HORI_ENCODERS, -HORI_POWER);
            }*/

            if (gamepad2.x) {
                setAllMovementMotorsPower(0);

                lastHoriPosition = horiSlide.getCurrentPosition();
                //intakeGrabber.setPosition(1);
                intakeGrabber.setPower(-grabberPower);
                sleep(1000);

                intakeGrabber.setPower(0);

                intakeFlipper.setPosition(intakeFlipperMacroPos);
                sleep(500);
                runMotorToEncoderPosition(horiSlide, (int) (-FULL_EXTENT_HORI_ENCODERS * 0.9), -horiMacroPower);
                intakeFlipper.setPosition(0);
                sleep(500);
                //intakeGrabber.setPosition(0);
                intakeGrabber.setPower(grabberPower);
                sleep(1000);
                intakeGrabber.setPower(0);
                intakeFlipper.setPosition(intakeFlipperMacroPos);
                sleep(500);

                runMotorToEncoderPosition(horiSlide, lastHoriPosition, horiMacroPower);
                intakeFlipper.setPosition(intakeFlipperMidPos);

            }

            telemetry.addData("Encoder: ", horiSlide.getCurrentPosition());
            telemetry.update();

        }

    }

    public void setAllMovementMotorsPower(float power) {
        frontLeftMotor.setPower(power);
        backLeftMotor.setPower(power);
        frontRightMotor.setPower(power);
        backRightMotor.setPower(power);
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

        //double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        double botHeading = myOtos.getPosition().h;

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
        motor.setTargetPosition(position);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(power);
        while (motor.isBusy() && opModeIsActive()) {
            telemetry.addData("Name: ", motor.getPortNumber());
            telemetry.addData("Pos: ", motor.getCurrentPosition());
            telemetry.update();
        }
        motor.setPower(0);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void scoreInitiate() {

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

        sleep(250);
        swing.setPosition(1);

    }

    private void configureOtos() {

        myOtos.setSignalProcessConfig(new SparkFunOTOS.SignalProcessConfig((byte) 0x0D));
        telemetry.addLine("Configuring OTOS...");
        telemetry.update();

        // Set the desired units for linear and angular measurements. Can be either
        // meters or inches for linear, and radians or degrees for angular. If not
        // set, the default is inches and degrees. Note that this setting is not
        // persisted in the sensor, so you need to set at the start of all your
        // OpModes if using the non-default value.
        // myOtos.setLinearUnit(DistanceUnit.METER);
        myOtos.setLinearUnit(DistanceUnit.INCH);
        // myOtos.setAngularUnit(AnguleUnit.RADIANS);
        myOtos.setAngularUnit(AngleUnit.RADIANS);

        // Assuming you've mounted your sensor to a robot and it's not centered,
        // you can specify the offset for the sensor relative to the center of the
        // robot. The units default to inches and degrees, but if you want to use
        // different units, specify them before setting the offset! Note that as of
        // firmware version 1.0, these values will be lost after a power cycle, so
        // you will need to set them each time you power up the sensor. For example, if
        // the sensor is mounted 5 inches to the left (negative X) and 10 inches
        // forward (positive Y) of the center of the robot, and mounted 90 degrees
        // clockwise (negative rotation) from the robot's orientation, the offset
        // would be {-5, 10, -90}. These can be any value, even the angle can be
        // tweaked slightly to compensate for imperfect mounting (eg. 1.3 degrees).
        SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(5.6, 0, 0);
        myOtos.setOffset(offset);

        // Here we can set the linear and angular scalars, which can compensate for
        // scaling issues with the sensor measurements. Note that as of firmware
        // version 1.0, these values will be lost after a power cycle, so you will
        // need to set them each time you power up the sensor. They can be any value
        // from 0.872 to 1.127 in increments of 0.001 (0.1%). It is recommended to
        // first set both scalars to 1.0, then calibrate the angular scalar, then
        // the linear scalar. To calibrate the angular scalar, spin the robot by
        // multiple rotations (eg. 10) to get a precise error, then set the scalar
        // to the inverse of the error. Remember that the angle wraps from -180 to
        // 180 degrees, so for example, if after 10 rotations counterclockwise
        // (positive rotation), the sensor reports -15 degrees, the required scalar
        // would be 3600/3585 = 1.004. To calibrate the linear scalar, move the
        // robot a known distance and measure the error; do this multiple times at
        // multiple speeds to get an average, then set the linear scalar to the
        // inverse of the error. For example, if you move the robot 100 inches and
        // the sensor reports 103 inches, set the linear scalar to 100/103 = 0.971
        myOtos.setLinearScalar(1.0);
        myOtos.setAngularScalar(1.0);

        // The IMU on the OTOS includes a gyroscope and accelerometer, which could
        // have an offset. Note that as of firmware version 1.0, the calibration
        // will be lost after a power cycle; the OTOS performs a quick calibration
        // when it powers up, but it is recommended to perform a more thorough
        // calibration at the start of all your OpModes. Note that the sensor must
        // be completely stationary and flat during calibration! When calling
        // calibrateImu(), you can specify the number of samples to take and whether
        // to wait until the calibration is complete. If no parameters are provided,
        // it will take 255 samples and wait until done; each sample takes about
        // 2.4ms, so about 612ms total
        myOtos.calibrateImu();

        // Reset the tracking algorithm - this resets the position to the origin,
        // but can also be used to recover from some rare tracking errors
        myOtos.resetTracking();

        // After resetting the tracking, the OTOS will report that the robot is at
        // the origin. If your robot does not start at the origin, or you have
        // another source of location information (eg. vision odometry), you can set
        // the OTOS location to match and it will continue to track from there.
        SparkFunOTOS.Pose2D currentPosition = new SparkFunOTOS.Pose2D(0, 0, 0);
        myOtos.setPosition(currentPosition);

        // Get the hardware and firmware version
        SparkFunOTOS.Version hwVersion = new SparkFunOTOS.Version();
        SparkFunOTOS.Version fwVersion = new SparkFunOTOS.Version();
        myOtos.getVersionInfo(hwVersion, fwVersion);

    }

}
