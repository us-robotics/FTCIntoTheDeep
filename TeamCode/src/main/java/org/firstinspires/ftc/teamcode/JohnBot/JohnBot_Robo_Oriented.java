package org.firstinspires.ftc.teamcode.JohnBot;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;


// Uses Field Centric Controlling
// The forward direction is defined as the FORWARD when the bot initializes
@TeleOp(name = "John Bot Robo-Oriented", group = "Linear OpMode")
@Disabled
public class JohnBot_Robo_Oriented extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Value Variables
        double flipperPower = 0.7;

        // Declare our motors
        // Make sure your ID's match your configuration

        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("left_front_drive");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("left_back_drive");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("right_front_drive");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("right_back_drive");
        DcMotor flipper = hardwareMap.dcMotor.get("flipper");

        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.


        frontLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        backRightMotor.setDirection(DcMotor.Direction.REVERSE);

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
            double max;

            double axial = gamepad1.left_stick_y;
            double lateral = -gamepad1.left_stick_x;
            double yaw = -gamepad1.right_stick_x;

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

            if (gamepad1.a) {
                flipper.setPower(flipperPower);
            } else if (gamepad1.b) {
                flipper.setPower(-flipperPower);
            } else {
                flipper.setPower(0);
            }

            // Control frontLeftMotor with gamepad1.a
            if (gamepad1.a) {
                frontLeftMotor.setPower(1);
            } else {
                frontLeftMotor.setPower(0);
            }

// Control frontRightMotor with gamepad1.b
            if (gamepad1.b) {
                frontRightMotor.setPower(1);
            } else {
                frontRightMotor.setPower(0);
            }

// Control backLeftMotor with gamepad1.x
            if (gamepad1.x) {
                backLeftMotor.setPower(1);
            } else {
                backLeftMotor.setPower(0);
            }

// Control backRightMotor with gamepad1.y
            if (gamepad1.y) {
                backRightMotor.setPower(1);
            } else {
                backRightMotor.setPower(0);
            }

        }
    }
}
