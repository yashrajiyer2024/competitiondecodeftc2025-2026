package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@TeleOp(name="AprilTag Vision + Dashboard Tester")
public class AprilTagVisionDashboardTester extends LinearOpMode {

    private AprilTag vision;

    @Override
    public void runOpMode() {

        telemetry.addLine("Initializing AprilTag Vision...");
        telemetry.update();

        // Create your AprilTag vision object
        vision = new AprilTag(hardwareMap, telemetry);

        telemetry.addLine("Vision Ready!");
        telemetry.addLine("Press PLAY to begin testing");
        telemetry.update();

        waitForStart();



        if (isStopRequested()) return;

        while (opModeIsActive()) {

            // Update the vision frame
            vision.update();

            // Get the most recent detection
            AprilTagDetection tag = vision.getLatestTag();

            // ------------------------------
            // DRIVER STATION TELEMETRY
            // ------------------------------
            vision.addTelemetry();
            telemetry.update();

            // ------------------------------
            // FTC DASHBOARD TELEMETRY
            // ------------------------------
            vision.addDashboardTelemetry(tag);

            sleep(50); // stabilizes data updates
        }

        // Turn off camera when done
        vision.cameraOff();
    }
}