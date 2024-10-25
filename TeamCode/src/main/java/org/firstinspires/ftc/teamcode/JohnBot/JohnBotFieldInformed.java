/* Copyright (c) 2023 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * JohnBotFieldInformed - TeleOp mode for JohnBot utilizing Limelight for AprilTag detection.
 * SWEENEY MODIFICATIONS
 * 9/25/24 - Switched motor power in move robot to negative, because 24885-bot needed it.
 * 9/25/24 - Integrated Limelight code and disabled old webcam code.
 */

package org.firstinspires.ftc.teamcode.JohnBot;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.limelightvision.LLFieldMap;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name = "John Bot (Field Informed)", group = "Autonomous")
@Disabled
public class JohnBotFieldInformed extends LinearOpMode {
    private static final double DESIRED_DISTANCE = 5.0; // Desired distance from target (inches)
    private static final double SPEED_GAIN = 0.02; // Forward speed control gain
    private static final double STRAFE_GAIN = 0.015; // Strafe speed control gain
    private static final double TURN_GAIN = 0.01; // Turn control gain
    private static final double MAX_AUTO_SPEED = 0.5; // Maximum speed for auto drive
    private static final double MAX_AUTO_STRAFE = 0.5; // Maximum strafe speed
    private static final double MAX_AUTO_TURN = 0.3; // Maximum turn speed
    private static final double MANUAL_TURN_MODIFIER = 2.0;
    private static final int DESIRED_TAG_ID = -1; // Desired tag ID (-1 for any tag)

    private DcMotor leftFrontDrive, rightFrontDrive, leftBackDrive, rightBackDrive;
    public BNO055IMU imu;
    private LLResultTypes.FiducialResult desiredTag;
    private Limelight3A limelight;

    @Override
    public void runOpMode() {
        initializeHardware();
        initializeLimelight();

        waitForStart();
        while (opModeIsActive()) {
            boolean targetFound = processLimelightData();
            double[] driveData;

            if (targetFound && gamepad1.left_bumper) {
                driveData = autoDrive();
            } else {
                driveData = manualDrive();
            }

            telemetry.update();
            moveRobot(driveData[0], driveData[1], driveData[2]);
            sleep(10);
        }
        limelight.stop();
    }

    private void initializeHardware() {
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");

        // Set motor directions
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu.initialize(parameters);
    }

    private void initializeLimelight() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

        telemetry.addData(">", "Robot Ready. Press Play.");
        telemetry.update();
    }

    private boolean processLimelightData() {
        limelight.updateRobotOrientation(imu.getAngularOrientation().firstAngle);
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            for (LLResultTypes.FiducialResult fr : result.getFiducialResults()) {
                telemetry.addData("ID", "ID: %d ", fr.getFiducialId());
            }

            Pose3D botpose_mt2 = result.getBotpose_MT2();
            if (botpose_mt2 != null) {
                double x = botpose_mt2.getPosition().x;
                double y = botpose_mt2.getPosition().y;
                telemetry.addData("MT2 Location:", "(" + x + ", " + y + ")");
            }

            for (LLResultTypes.FiducialResult fr : result.getFiducialResults()) {
                if (fr.getFiducialId() == DESIRED_TAG_ID || DESIRED_TAG_ID < 0) {
                    desiredTag = fr;
                    return true;
                }
            }

        } else {
            telemetry.addData("Limelight", "No data available");
        }
        return false;
    }


    private double[] autoDrive() {
        LLResult result = limelight.getLatestResult();
        Pose3D botpose = result.getBotpose();
        YawPitchRollAngles orientation = botpose.getOrientation();

        double range = 100-result.getTa();
        double yaw = orientation.getYaw();
        double bearing = -desiredTag.getTargetXDegrees();

        // Compute the drive power based on target range and error
        double rangeError = (range - DESIRED_DISTANCE);
        double headingError = bearing;
        double yawError = yaw;

        double drive = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
        double turn = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
        double strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

        telemetry.addData("Auto (Data)", "Range %5.2f, Yaw %5.2f, Bearing %5.2f ", range, yaw, bearing);
        telemetry.addData("Auto", "Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
        return new double[]{drive, turn, strafe}; // Modify to return all three values if necessary
    }

    private double[] manualDrive() {
        double drive = -gamepad1.left_stick_y / 2.0; // Reduce drive rate to 50%
        double strafe = -gamepad1.left_stick_x / 2.0; // Reduce strafe rate to 50%
        double turn = -gamepad1.right_stick_x / 1.5; // Reduce turn rate to something
        telemetry.addData("Manual", "Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
        return new double[]{drive, strafe, turn}; // Modify to return all three values if necessary
    }

    public void moveRobot(double x, double y, double yaw) {
        // Calculate wheel powers
        double leftFrontPower = x - y - yaw;
        double rightFrontPower = x + y + yaw;
        double leftBackPower = x + y - yaw;
        double rightBackPower = x - y + yaw;

        // Apply the Sweeney modification
        leftFrontPower *= -1;
        rightFrontPower *= -1;
        leftBackPower *= -1;
        rightBackPower *= -1;

        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.abs(leftFrontPower), Math.max(Math.abs(rightFrontPower), Math.max(Math.abs(leftBackPower), Math.abs(rightBackPower))));
        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        // Set motor powers
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);
    }
}
