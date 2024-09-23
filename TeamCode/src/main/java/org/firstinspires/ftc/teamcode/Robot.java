package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

public class Robot {

    /* Declare OpMode members. */
    public DcMotor frontLeft = null;
    public DcMotor backLeft = null;
    public DcMotor frontRight = null;
    public DcMotor backRight = null;
    public DcMotor spool = null;
    public DcMotor lift = null;
    public Servo leftIntake = null;
    public Servo rightIntake = null;
    public Servo planeLift = null;
    public Servo planeLaunch = null;
    public Servo yellowArm = null;
    public Servo yellowDrop = null;
    public BNO055IMU imu;

    OpenCvWebcam webcam;

    HardwareMap hardwareMap = null;

    public void init(HardwareMap h) {
        hardwareMap = h;
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam");
        webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
    }

}