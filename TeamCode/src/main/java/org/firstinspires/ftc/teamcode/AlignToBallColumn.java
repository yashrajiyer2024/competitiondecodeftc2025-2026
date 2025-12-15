package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.vision.BallColumnPipeline;
import org.openftc.easyopencv.*;
import com.acmerobotics.dashboard.config.Config;

@Config
@Autonomous(name = "Ball Column Detection")
public class AlignToBallColumn extends LinearOpMode {

    // Drivetrain
    private DcMotor LB, LF, RB, RF;

    // Shooter + Intake
    private DcMotorEx LSX, RSX, IntakeEx;

    // Vision
    private AprilTag vision;

    // Constants
    private static final double DRIVE_FWD = 0.5;
    public static double backDrive = 0.3;
    public static double frontDrive = 0.5;
    private static final double DRIVE_TURN = 0.2;

    private static final double SHOOTER_RPM = 1702;
    private static final double INTAKE_RPM = 1000;

    OpenCvCamera webcam;
    BallColumnPipeline pipeline;

    @Override
    public void runOpMode() throws InterruptedException {

        // ---------------------------
        // Hardware Initialization
        // ---------------------------
        LB = hardwareMap.get(DcMotor.class, "LB");
        LF = hardwareMap.get(DcMotor.class, "LF");
        RB = hardwareMap.get(DcMotor.class, "RB");
        RF = hardwareMap.get(DcMotor.class, "RF");

        LSX = hardwareMap.get(DcMotorEx.class, "LS");
        RSX = hardwareMap.get(DcMotorEx.class, "RS");
        IntakeEx = hardwareMap.get(DcMotorEx.class, "Intake");
        LB.setDirection(DcMotor.Direction.FORWARD);
        LF.setDirection(DcMotor.Direction.REVERSE);
        RB.setDirection(DcMotor.Direction.FORWARD);
        RF.setDirection(DcMotor.Direction.FORWARD);

        LSX.setDirection(DcMotorEx.Direction.REVERSE);
        RSX.setDirection(DcMotorEx.Direction.FORWARD);
        IntakeEx.setDirection(DcMotorEx.Direction.REVERSE);

        // ---- Webcam initialization ----
        WebcamName cameraName = hardwareMap.get(WebcamName.class, "WebcamBALLS");

        int cameraMonitorId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        webcam = OpenCvCameraFactory.getInstance().createWebcam(cameraName, cameraMonitorId);

        // Attach pipeline
        pipeline = new BallColumnPipeline();
        webcam.setPipeline(pipeline);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera failed to open!", errorCode);
                telemetry.update();
            }
        });

        // Stream to FTC Dashboard
        com.acmerobotics.dashboard.FtcDashboard.getInstance().startCameraStream(webcam, 30);

        telemetry.addLine("Ready. Press start.");
        telemetry.update();
        waitForStart();

        // ---- Main Loop ----
        while (opModeIsActive()) {

            telemetry.addData("Found Column", pipeline.foundColumn);
            telemetry.addData("Column X", pipeline.columnCenterX);
            telemetry.addData("Horizontal Error", pipeline.error);
            telemetry.addData("BackDrive:", backDrive);

            // ---- Alignment Code ----
            double threshold = -166;  // acceptable alignment error in pixels

            if (pipeline.foundColumn) {
                if (pipeline.error > threshold) {
                    // Strafe RIGHT using corrected strafe ratios
                    drive(frontDrive, -backDrive, -frontDrive, backDrive);
                }
                else if (pipeline.error < -threshold) {
                    // Strafe LEFT using corrected strafe ratios
                    drive(-frontDrive, backDrive, frontDrive, -backDrive);
                }
                else {
                    drive(0, 0, 0, 0);
                }
            } else {
                // No column detected
                drive(0, 0, 0, 0);
            }

            telemetry.update();
        }
    }

    private void drive(double lf, double lb, double rf, double rb) {
        LF.setPower(lf);
        LB.setPower(lb);
        RF.setPower(rf);
        RB.setPower(rb);
    }
}
