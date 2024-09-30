package org.firstinspires.ftc.teamcode.JohnBot;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name="CenterStage", group="Linear Opmode")
@Disabled
public class CenterStage extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

    }

    public enum Location{
        LEFT,MIDDLE,RIGHT
    }
    public static Robot robot = new Robot();
    public static Location duckLocation;

    public void startStream(String color){

        robot.webcam.openCameraDeviceAsync( new OpenCvCamera.AsyncCameraOpenListener( ) {
            @Override
            public void onOpened(){
                robot.webcam.startStreaming( 320, 240, OpenCvCameraRotation.UPRIGHT );
            }

            @Override
            public void onError(int errorCode){
                Log.e( "CAMERA_DEVICE", "Camera could not be opened. Error code: " + errorCode );
            }
        } );
    }

    public void stopStream(){
        robot.webcam.stopStreaming();
    }

}
