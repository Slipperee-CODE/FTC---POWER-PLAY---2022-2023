package org.firstinspires.ftc.teamcode;

/* Copyright (c) 2017 FIRST. All rights reserved.
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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="TheGoatedWoatedAutonomousModed", group="Iterative Autonomous")
//@Disabled
public class TheGoatedWoatedAutonomousModed extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor RightFront = null;
    private DcMotor RightBack = null;
    private DcMotor LeftFront = null;
    private DcMotor LeftBack = null;

    private float SpeedReduction = 50;

    OpenCvWebcam webcam1 = null;

    int colorIAmSeeing = 0;

    double ractualAvg;
    double gactualAvg;
    double bactualAvg;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        LeftFront = hardwareMap.dcMotor.get("LeftFront");
        LeftBack = hardwareMap.dcMotor.get("LeftBack");
        RightFront = hardwareMap.dcMotor.get("RightFront");
        RightBack = hardwareMap.dcMotor.get("RightBack");

        SpeedReduction = SpeedReduction/100;


        WebcamName webcamName = hardwareMap.get(WebcamName.class, "webcam1");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam1 = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "webcam1"), cameraMonitorViewId);

        webcam1.setPipeline(new examplePipeline());

        webcam1.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam1.startStreaming(640, 360, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });


        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        // Reverse the right side motors
        // Reverse left motors if you are using NeveRests
        RightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        RightBack.setDirection(DcMotorSimple.Direction.REVERSE);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {

        telemetry.addData("R", ractualAvg);
        telemetry.addData("Y", gactualAvg);
        telemetry.addData("B", bactualAvg);

        if (ractualAvg > 15 || gactualAvg > 15 || bactualAvg > 15){
            if (ractualAvg > gactualAvg && ractualAvg > bactualAvg){
                colorIAmSeeing = 1;
            }
            else if (gactualAvg > ractualAvg && gactualAvg > bactualAvg)
            {
                colorIAmSeeing = 2;
            }
            else if (bactualAvg > ractualAvg && bactualAvg > gactualAvg){
                colorIAmSeeing = 3;
            }
            else{
                colorIAmSeeing = 0;
            }
        }

        if (colorIAmSeeing == 1){
            telemetry.addData("Green", colorIAmSeeing);
        }
        else if (colorIAmSeeing == 2){
            telemetry.addData("Yellow", colorIAmSeeing);
        }
        else if (colorIAmSeeing == 3){
            telemetry.addData("Blue", colorIAmSeeing);
        }
        else{
            telemetry.addData("No Color :( ", colorIAmSeeing);
        }

        telemetry.update();
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();



    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        if (colorIAmSeeing == 1){
            double power = 0.7;
            LeftFront.setPower(power);
            LeftBack.setPower(-power);
            RightFront.setPower(-power);
            RightBack.setPower(power);
            if (runtime.milliseconds() <= 300.0){
                LeftFront.setPower(0);
                LeftBack.setPower(0);
                RightFront.setPower(0);
                RightBack.setPower(0);
            }
        }
        telemetry.update();
        //Main loop
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {

        //Stuff for when the program stops

    }

    public class examplePipeline extends OpenCvPipeline{

        public Mat processFrame(Mat input) {
            // "Mat" stands for matrix, which is basically the image that the detector will process
            // the input matrix is the image coming from the camera
            // the function will return a matrix to be drawn on your phone's screen

            // The detector detects regular stones. The camera fits two stones.
            // If it finds one regular stone then the other must be the skystone.
            // If both are regular stones, it returns NONE to tell the robot to keep looking

            // Make a working copy of the input matrix in HSV
            Mat mat = new Mat();
            Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);

            // We create a HSV range for yellow to detect regular stones
            // NOTE: In OpenCV's implementation,
            // Hue values are half the real value


            //RED SECTION//

            Scalar rlowHSV = new Scalar(60, 100, 100); // lower bound HSV for green
            Scalar rhighHSV = new Scalar(71, 255, 255); // higher bound HSV for green
            Mat thresh = new Mat();

            // We'll get a black and white image. The white regions represent the regular stones.
            // inRange(): thresh[i][j] = {255,255,255} if mat[i][i] is within the range
            Core.inRange(mat, rlowHSV, rhighHSV, thresh);

            Scalar ravg = Core.mean(thresh);

            ractualAvg = ravg.val[0];

            //telemetry.addData("Avg Red Present : ", ractualAvg);


            //YELLOW SECTION//

            Scalar glowHSV = new Scalar(20, 100, 100); // lower bound HSV for yellow
            Scalar ghighHSV = new Scalar(40, 255, 255); // higher bound HSV for yellow
            thresh = new Mat();

            // We'll get a black and white image. The white regions represent the regular stones.
            // inRange(): thresh[i][j] = {255,255,255} if mat[i][i] is within the range
            Core.inRange(mat, glowHSV,ghighHSV, thresh);

            Scalar gavg = Core.mean(thresh);

            gactualAvg = gavg.val[0];

            //telemetry.addData("Avg Green Present : ", gactualAvg);


            //BLUE SECTION//

            Scalar blowHSV = new Scalar(96, 100, 100); // lower bound HSV for yellow
            Scalar bhighHSV = new Scalar(121, 255, 255); // higher bound HSV for yellow
            thresh = new Mat();

            // We'll get a black and white image. The white regions represent the regular stones.
            // inRange(): thresh[i][j] = {255,255,255} if mat[i][i] is within the range
            Core.inRange(mat, blowHSV, bhighHSV, thresh);

            Scalar bavg = Core.mean(thresh);

            bactualAvg = bavg.val[0];

            //telemetry.addData("Avg Blue Present : ", bactualAvg);




            return thresh;
        }
    }
}


