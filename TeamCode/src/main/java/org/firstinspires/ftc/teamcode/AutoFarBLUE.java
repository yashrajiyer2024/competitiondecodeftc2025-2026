package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@Autonomous(name="AutoFarBLUE")
public class AutoFarBLUE extends LinearOpMode {

    // Drivetrain
    private DcMotor LB, LF, RB, RF;

    // Shooter + Intake
    private DcMotorEx LSX, RSX, IntakeEx;

    // Vision
    private AprilTag vision;

    // Constants
    private static final double DRIVE_FWD = 0.5;
    private static final double DRIVE_TURN = 0.2;

    private static final double SHOOTER_RPM = 1702;
    private static final double INTAKE_RPM = 1000;

    // Targeting constants
    private static final int TARGET_TAG_ID = 20;   // Blue GOAL TAG
    private static final double TARGET_DISTANCE_IN = 5495;
    private static final double TARGET_RAW_X = -2.13;
    private static final double RAW_X_TOLERANCE = 0.2;

    @Override
    public void runOpMode() throws InterruptedException {

        // ---------------------------
        // Hardware Init
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

        // Vision
        telemetry.addLine("Initializing AprilTag...");
        telemetry.update();
        vision = new AprilTag(hardwareMap, telemetry);
        telemetry.addLine("Vision Ready");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // ---------------------------
        // STEP 1 — DRIVE FORWARD UNTIL DISTANCE MET
        // ---------------------------
        drive1(DRIVE_FWD, DRIVE_FWD, DRIVE_FWD, DRIVE_FWD, 150);
        sleep(1200);

        // Small pause
        sleep(50);

        // ---------------------------
        // STEP 2 — TURN UNTIL rawPose.x ALIGNED
        // ---------------------------
        while (opModeIsActive()) {

            vision.update();
            AprilTagDetection tag = vision.getTagById(TARGET_TAG_ID);

            if (tag != null && tag.rawPose != null) {

                double rawX = tag.rawPose.x;
                telemetry.addData("rawX", rawX);

                if (Math.abs(rawX - TARGET_RAW_X) <= RAW_X_TOLERANCE) {
                    drive(0, 0, 0, 0);
                    telemetry.addLine("Aligned (rawX in tolerance).");
                    telemetry.update();
                    break;
                }

                if (rawX > TARGET_RAW_X) {
                    telemetry.addLine("Turning LEFT");
                    drive1(DRIVE_TURN, DRIVE_TURN, -DRIVE_TURN,-DRIVE_TURN, 5);
                } else {
                    telemetry.addLine("Turning RIGHT");
                    drive1(-DRIVE_TURN, -DRIVE_TURN, DRIVE_TURN, DRIVE_TURN, 5);
                }
            }
            else {
                drive(0, 0, 0, 0);
            }

            telemetry.update();
        }

        // ---------------------------
        // STEP 3 — SHOOT SEQUENCE
        // ---------------------------
        LSX.setVelocity(SHOOTER_RPM);
        RSX.setVelocity(SHOOTER_RPM);
        sleep(3000);

        IntakeEx.setVelocity(INTAKE_RPM);
        sleep(1500);
        IntakeEx.setVelocity(0);
        IntakeEx.setVelocity(INTAKE_RPM-5);
        sleep(1500);
        IntakeEx.setVelocity(0);

        LSX.setVelocity(0);
        RSX.setVelocity(0);

        drive1(DRIVE_FWD, DRIVE_FWD, DRIVE_FWD, DRIVE_FWD, 800);

        stopAll();
    }

    // ================== Helper Functions ==================

    private void stopAll() {
        LF.setPower(0);
        LB.setPower(0);
        RF.setPower(0);
        RB.setPower(0);
        LSX.setVelocity(0);
        RSX.setVelocity(0);
        IntakeEx.setVelocity(0);
    }

    private void drive(double lf, double lb, double rf, double rb) {
        LF.setPower(lf);
        LB.setPower(lb);
        RF.setPower(rf);
        RB.setPower(rb);
    }

    private void drive1(double lf, double lb, double rf, double rb, long timeMs) {
        LF.setPower(lf);
        LB.setPower(lb);
        RF.setPower(rf);
        RB.setPower(rb);
        sleep(timeMs);
        LF.setPower(0);
        LB.setPower(0);
        RF.setPower(0);
        RB.setPower(0);
    }
}
