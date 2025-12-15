package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

/**
 * Improved AutoClose:
 * - Waits for a stable tag reading
 * - Backs up until smoothed distance > targetDist
 * - Ensures shooter spin-up (checks velocity) before feeding
 * - Safety timeouts everywhere
 * - ADDED: constant backup speed (-0.55)
 * - ADDED: instant brake when tag reached
 * - REPLACED: final 7-inch strafe -> now: CCW ~100° turn + forward drive
 */
@Autonomous(name="AutoCloseBlue")
public class AutoCloseBlue extends LinearOpMode {

    // Drive motors
    private DcMotor LB, LF, RB, RF;

    // Vision
    private AprilTag vision;

    // Shooter + Intake
    private DcMotorEx LSX, RSX, IntakeEx;

    // Constants
    private static final double DRIVE_FWD = 0.70;
    private static final double SHOOTER_RPM = 1450;
    private static final double INTAKE_RPM1 = 1350;
    private static final double INTAKE_RPM2 = 800;

    private static final double TARGET_DIST_IN = 2450;

    @Override
    public void runOpMode() throws InterruptedException {

        // === Hardware Map ===
        LB = hardwareMap.get(DcMotor.class, "LB");
        LF = hardwareMap.get(DcMotor.class, "LF");
        RB = hardwareMap.get(DcMotor.class, "RB");
        RF = hardwareMap.get(DcMotor.class, "RF");
        LSX = hardwareMap.get(DcMotorEx.class, "LS");
        RSX = hardwareMap.get(DcMotorEx.class, "RS");
        IntakeEx = hardwareMap.get(DcMotorEx.class, "Intake");

        // Directions
        LB.setDirection(DcMotor.Direction.FORWARD);
        LF.setDirection(DcMotor.Direction.REVERSE);
        RB.setDirection(DcMotor.Direction.FORWARD);
        RF.setDirection(DcMotor.Direction.FORWARD);
        LSX.setDirection(DcMotorEx.Direction.REVERSE);
        RSX.setDirection(DcMotorEx.Direction.FORWARD);
        IntakeEx.setDirection(DcMotorEx.Direction.REVERSE);

        LSX.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RSX.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        IntakeEx.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addLine("Initializing AprilTag Vision...");
        telemetry.update();

        vision = new AprilTag(hardwareMap, telemetry);

        telemetry.addLine("Vision Ready!");
        telemetry.addLine("Press PLAY to begin");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // ===============================
        // PHASE A – WAIT FOR STABLE TAG
        // ===============================
        telemetry.addLine("Phase A: waiting for stable tag...");
        telemetry.update();

        double startA = getRuntime();
        double timeoutA = 1;

        while (opModeIsActive()) {
            vision.update();
            AprilTagDetection tag = vision.getLatestTag();
            vision.addTelemetry();
            vision.addDashboardTelemetry(tag);
            telemetry.update();

            if (tag != null && vision.isStable()) {
                telemetry.addLine("TAG STABLE — applying brake pulse");
                telemetry.update();

                drive(+0.4, +0.4, +0.4, +0.4);
                sleep(120);
                drive(0,0,0,0);
                sleep(150);
                break;
            }

            if (getRuntime() - startA > timeoutA) {
                telemetry.addLine("Timeout waiting for stable tag — proceeding");
                telemetry.update();
                break;
            }

            sleep(30);
        }

        // ===============================================
        // PHASE B – BACK UP CONSTANT SPEED until distance
        // ===============================================
        telemetry.addLine("Phase B: backing up until distance > " + TARGET_DIST_IN);
        telemetry.update();

        double startB = getRuntime();
        double timeoutB = 6.0;

        while (opModeIsActive()) {

            vision.update();
            AprilTagDetection tag = vision.getLatestTag();

            double smoothedDist = vision.getSmoothedDistanceInches();

            if (smoothedDist > 0) {
                telemetry.addData("SmoothedDist(in)", "%.2f", smoothedDist);

                if (smoothedDist > TARGET_DIST_IN) {
                    // INSTANT BRAKE
                    drive(0,0,0,0);
                    sleep(120);

                    telemetry.addLine("Reached target distance.");
                    telemetry.update();
                    break;
                } else {
                    drive(-0.55, -0.55, -0.55, -0.55);
                }
            } else {
                drive(-0.55, -0.55, -0.55, -0.55);
            }

            vision.addTelemetry();
            vision.addDashboardTelemetry(tag);
            telemetry.update();

            if (getRuntime() - startB > timeoutB) {
                telemetry.addLine("Backup timeout — moving to shoot");
                telemetry.update();
                drive(0,0,0,0);
                break;
            }

            sleep(30);
        }

        drive(0,0,0,0);
        sleep(100);

        // =====================
        // SHOOTER ROUTINE
        // =====================
        telemetry.addLine("Spinning shooter...");
        telemetry.update();

        LSX.setVelocity(SHOOTER_RPM);
        RSX.setVelocity(SHOOTER_RPM);

        double shooterStart = getRuntime();
        double shooterTimeout = 5.0;
        boolean shooterReady = false;

        while (opModeIsActive() && (getRuntime() - shooterStart < shooterTimeout)) {
            try {
                double vLS = Math.abs(LSX.getVelocity());
                double vRS = Math.abs(RSX.getVelocity());
                telemetry.addData("LS vel", "%.0f", vLS);
                telemetry.addData("RS vel", "%.0f", vRS);

                if (vLS >= 0.90 * SHOOTER_RPM && vRS >= 0.90 * SHOOTER_RPM) {
                    shooterReady = true;
                    break;
                }
            } catch (Exception e) {}

            telemetry.update();
            sleep(150);
        }

        if (!shooterReady) {
            telemetry.addLine("Shooter timed spin-up fallback");
            telemetry.update();
            sleep(1500);
        } else {
            telemetry.addLine("Shooter ready");
            telemetry.update();
            sleep(200);
        }

        telemetry.addLine("Feeding...");
        telemetry.update();

        IntakeEx.setVelocity(INTAKE_RPM1);
        sleep(1500);
        IntakeEx.setVelocity(0);
        sleep(200);
        IntakeEx.setVelocity(INTAKE_RPM2);
        sleep(1500);
        IntakeEx.setVelocity(0);

        LSX.setVelocity(0);
        RSX.setVelocity(0);

        // =====================================================
        // FINAL: COUNTER-CLOCKWISE ~100° TURN + 1 sec forward
        // =====================================================

        // CCW turn: left motors reverse, right motors forward
        double turnPower = 0.5;
        long turnMs = 700;    // same timing as before (~100°)
        drive(-turnPower, -turnPower, turnPower, turnPower);
        sleep(turnMs);

        drive(0,0,0,0);
        sleep(150);

        // Forward drive for 1 second
        drive(DRIVE_FWD, DRIVE_FWD, DRIVE_FWD, DRIVE_FWD);
        sleep(750);

        drive(0,0,0,0);
        sleep(150);

        // End
        stopAll();
        telemetry.addLine("AutoClose complete");
        telemetry.update();
    }

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
}
