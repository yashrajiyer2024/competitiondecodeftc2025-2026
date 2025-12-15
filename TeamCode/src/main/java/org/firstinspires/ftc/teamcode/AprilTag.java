package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

/**
 * Robust AprilTag helper:
 * - Uses detection.rawPose when available for accurate distance
 * - Exponential smoothing for distance and center X
 * - Stability counter (consecutive stable frames)
 * - Dashboard telemetry (smoothed values + raw pose)
 */
public class AprilTag {
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private Telemetry telemetry;

    private List<AprilTagDetection> currentDetections;

    // Camera resolution constants
    private static final int CAMERA_WIDTH = 640;
    private static final int CAMERA_HEIGHT = 480;

    // Tag size in inches (kept for reference if needed)
    private static final double TAG_SIZE_IN = 6.5;

    // Smoothing alpha for exponential moving average (0 < alpha <= 1)
    private static final double SMOOTH_ALPHA = 0.25;

    // How many consecutive stable frames are required before we treat detection as stable
    private static final int STABILITY_REQUIRED = 3;

    // Holds goal label if found
    public String goalLabel = "None";
    public String patternLabel = "None";

    // Smoothed values (in inches for distance, px for centerX)
    private double smoothedDistanceIn = -1;
    private double smoothedCenterX = -1;

    // Stability counter
    private int stableFrames = 0;

    public AprilTag(HardwareMap hwMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        // Build the AprilTag processor
        aprilTag = new AprilTagProcessor.Builder().build();
        aprilTag.setDecimation(1);

        // Open camera and start vision with locked resolution
        visionPortal = new VisionPortal.Builder()
                .setCamera(hwMap.get(WebcamName.class, "Webcam"))
                .setCameraResolution(new Size(CAMERA_WIDTH, CAMERA_HEIGHT))
                .addProcessor(aprilTag)
                .enableLiveView(true)
                .build();

        // Start dashboard stream (safe to call here)
        try {
            FtcDashboard.getInstance().startCameraStream(visionPortal, 30);
        } catch (Exception e) {
            telemetry.addData("Dashboard", "startCameraStream failed: " + e.getMessage());
            telemetry.update();
        }
    }

    /**
     * Must be called frequently (20-50ms) to keep detections current.
     */
    public void update() {
        currentDetections = aprilTag.getDetections();

        // Reset labels every update (will be set if tag found)
        goalLabel = "None";
        patternLabel = "None";

        if (currentDetections != null && !currentDetections.isEmpty()) {
            // Use the first detection for simple autos; you can also search by ID
            AprilTagDetection detection = currentDetections.get(0);

            // Labels
            switch (detection.id) {
                case 20:
                    goalLabel = "blueGoal";
                    break;
                case 24:
                    goalLabel = "redGoal";
                    break;
                case 21:
                    patternLabel = "GPP";
                    break;
                case 22:
                    patternLabel = "PGP";
                    break;
                case 23:
                    patternLabel = "PPG";
                    break;
                default:
                    break;
            }

            // Compute distance using rawPose if available (meters -> inches)
            double distanceInches = -1;
            if (detection.rawPose != null) {
                double x = detection.rawPose.x;
                double y = detection.rawPose.y;
                double z = detection.rawPose.z;
                double distMeters = Math.sqrt(x * x + y * y + z * z);
                distanceInches = distMeters * 39.37007874; // meters -> inches
            } else {
                // fallback: approximate using center/corners if rawPose unavailable
                if (detection.corners != null && detection.corners.length >= 2) {
                    double pixelWidth = Math.abs(detection.corners[0].x - detection.corners[1].x);
                    if (pixelWidth > 0) {
                        // NOTE: this is a fallback and will be less accurate
                        double focalPx = 800; // conservative fallback if no calibration
                        distanceInches = (TAG_SIZE_IN * focalPx) / pixelWidth;
                    }
                }
            }

            // Center X in pixels if available
            double centerX = (detection.center != null) ? detection.center.x : (CAMERA_WIDTH / 2.0);

            // Initialize smoothed values if first valid reading
            if (smoothedDistanceIn < 0) {
                smoothedDistanceIn = distanceInches;
            } else if (distanceInches > 0) {
                smoothedDistanceIn = SMOOTH_ALPHA * distanceInches + (1 - SMOOTH_ALPHA) * smoothedDistanceIn;
            }

            if (smoothedCenterX < 0) {
                smoothedCenterX = centerX;
            } else {
                smoothedCenterX = SMOOTH_ALPHA * centerX + (1 - SMOOTH_ALPHA) * smoothedCenterX;
            }

            // Update stability counter (consider stable if change small)
            boolean frameStable = false;
            if (distanceInches > 0) {
                double distDelta = Math.abs(distanceInches - smoothedDistanceIn);
                double centerDelta = Math.abs(centerX - smoothedCenterX);

                // thresholds tuned to be forgiving but filter spikes
                frameStable = (distDelta < 3.0) && (centerDelta < 15.0); // inches, pixels
            }

            if (frameStable) {
                stableFrames = Math.min(stableFrames + 1, STABILITY_REQUIRED);
            } else {
                stableFrames = 0;
            }

        } else {
            // No detection: decay smoothed values slowly so we don't get huge spikes when tag briefly disappears
            stableFrames = 0;
            // optional slow decay: leave smoothed values as-is (so code can use last known) or set to -1 to force reacquire
            // smoothedDistanceIn = -1;
            // smoothedCenterX = -1;
        }
    }

    /**
     * Put human-friendly telemetry to driver station (uses the underlying telemetry object).
     */
    public void addTelemetry() {
        if (currentDetections != null && !currentDetections.isEmpty()) {
            for (AprilTagDetection detection : currentDetections) {
                telemetry.addData("AprilTag ID", detection.id);
                telemetry.addData("Goal Label", goalLabel);
                telemetry.addData("Pattern Label", patternLabel);

                if (detection.rawPose != null) {
                    telemetry.addData("Raw X (m)", "%.3f", detection.rawPose.x);
                    telemetry.addData("Raw Y (m)", "%.3f", detection.rawPose.y);
                    telemetry.addData("Raw Z (m)", "%.3f", detection.rawPose.z);
                } else {
                    telemetry.addLine("Raw Pose: null");
                }

                telemetry.addData("Smoothed Dist (in)", "%.2f", smoothedDistanceIn);
                telemetry.addData("Smoothed CenterX (px)", "%.1f", smoothedCenterX);
                telemetry.addData("StableFrames", stableFrames);
            }
        } else {
            telemetry.addLine("No AprilTags detected");
        }
    }

    /**
     * Sends a structured packet to FTC Dashboard. Call this each loop after update().
     */
    public void addDashboardTelemetry(AprilTagDetection detection) {
        TelemetryPacket packet = new TelemetryPacket();

        if (detection != null) {
            packet.put("Tag ID", detection.id);
            packet.put("Goal Label", goalLabel);
            packet.put("Pattern Label", patternLabel);
            packet.put("Smoothed Distance (in)", smoothedDistanceIn);
            packet.put("Smoothed CenterX (px)", smoothedCenterX);
            packet.put("StableFrames", stableFrames);

            if (detection.rawPose != null) {
                packet.put("RawPose X (m)", detection.rawPose.x);
                packet.put("RawPose Y (m)", detection.rawPose.y);
                packet.put("RawPose Z (m)", detection.rawPose.z);
            } else {
                packet.put("RawPose", "null");
            }
        } else {
            packet.put("Tag", "NO DETECTION");
            packet.put("Smoothed Distance (in)", smoothedDistanceIn);
            packet.put("Smoothed CenterX (px)", smoothedCenterX);
            packet.put("StableFrames", stableFrames);
        }

        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }

    /** Returns the first detection, or null if none */
    public AprilTagDetection getLatestTag() {
        if (currentDetections != null && !currentDetections.isEmpty()) {
            return currentDetections.get(0);
        }
        return null;
    }

    /**
     * Return the *last-smoothed* distance in inches (may be -1 if never set).
     */
    public double getSmoothedDistanceInches() {
        return smoothedDistanceIn;
    }

    /**
     * Return the *last-smoothed* center X in px (may be -1 if never set).
     */
    public double getSmoothedCenterX() {
        return smoothedCenterX;
    }

    /** Calculates distance in inches for a given detection (best effort) */
    public double getDistanceInches(AprilTagDetection detection) {
        if (detection == null) return -1;
        if (detection.rawPose != null) {
            double x = detection.rawPose.x;
            double y = detection.rawPose.y;
            double z = detection.rawPose.z;
            double distMeters = Math.sqrt(x * x + y * y + z * z);
            return distMeters * 39.37007874;
        } else {
            // fallback to pixel calculation (less accurate)
            if (detection.corners != null && detection.corners.length >= 2) {
                double pixelWidth = Math.abs(detection.corners[0].x - detection.corners[1].x);
                if (pixelWidth <= 0) return -1;
                double focalPx = 800;
                return (TAG_SIZE_IN * focalPx) / pixelWidth;
            }
        }
        return -1;
    }

    /** Returns the image width in pixels (needed for centering) */
    public int getImageWidth() {
        return CAMERA_WIDTH;
    }

    public void cameraOff() {
        if (visionPortal != null) visionPortal.close();
    }

    /** Returns horizontal offset in pixels (positive = right, negative = left) using smoothed center */
    public double getPixelOffsetSmoothed(AprilTagDetection detection) {
        double midpoint = CAMERA_WIDTH / 2.0;
        if (smoothedCenterX < 0) {
            if (detection != null && detection.center != null) {
                return detection.center.x - midpoint;
            }
            return 0;
        } else {
            return smoothedCenterX - midpoint;
        }
    }

    public VisionPortal getCamera() {
        return visionPortal;
    }

    /**
     * Returns true if we have had enough consecutive stable frames to act (detection is stable)
     */
    public boolean isStable() {
        return stableFrames >= STABILITY_REQUIRED;
    }

    public AprilTagDetection getTagById(int targetId) {
        if (currentDetections == null) return null;
        for (AprilTagDetection detection : currentDetections) {
            if (detection.id == targetId) return detection;
        }
        return null; // not found
    }
}
