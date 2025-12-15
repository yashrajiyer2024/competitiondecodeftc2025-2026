package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Autonomous(name = "TriangleAutoBlue", group = "Linear OpMode")
public class TriangleAutoBlue extends LinearOpMode {

    // Drive motors
    private DcMotor LB, LF, RB, RF;

    // Shooter + Intake
    private DcMotorEx LSX, RSX, IntakeEx;

    // Constants
    private static final double DRIVE_FWD = 0.75;
    private static final double DRIVE_TURN = .75;
    private static final double SHOOTER_RPM = 1780;
    private static final double INTAKE_RPM = 1000;

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

        // === Motor Directions ===
        LB.setDirection(DcMotor.Direction.FORWARD);
        LF.setDirection(DcMotor.Direction.REVERSE);
        RB.setDirection(DcMotor.Direction.FORWARD);
        RF.setDirection(DcMotor.Direction.FORWARD);

        LSX.setDirection(DcMotorEx.Direction.REVERSE);
        RSX.setDirection(DcMotorEx.Direction.FORWARD);
        IntakeEx.setDirection(DcMotorEx.Direction.REVERSE);

        waitForStart();
        if (isStopRequested()) return;

        // ---------------------------------------------------------------------
        // 1️⃣ DRIVE FORWARD ~5.5 ft
        // ---------------------------------------------------------------------
        // Previously 950ms for ~2.5 ft → scale by 5.5 / 2.5 = 2.2x
        drive(DRIVE_FWD, DRIVE_FWD, DRIVE_FWD, DRIVE_FWD, 150);

        // ---------------------------------------------------------------------
        // 2️⃣ CLOCKWISE TURN ~150°
        // ---------------------------------------------------------------------
        // Your previous 80° turn was 700ms
        // Scale factor: 150 / 80 = 1.875 → 700 * 1.875 = 1312 ms
        drive(-DRIVE_TURN, -DRIVE_TURN, DRIVE_TURN, DRIVE_TURN, 85);

        // ---------------------------------------------------------------------
        // 3️⃣ Spin SHOOTER UP (1400 RPM) for 1 sec
        // ---------------------------------------------------------------------
        LSX.setVelocity(SHOOTER_RPM);
        RSX.setVelocity(SHOOTER_RPM);
        sleep(1000);

        // ---------------------------------------------------------------------
        // 4️⃣ Run INTAKE for 3 sec WHILE shooter stays running
        // ---------------------------------------------------------------------
        IntakeEx.setVelocity(INTAKE_RPM);
        sleep(3000);

        // Stop intake
        IntakeEx.setVelocity(0);

        // ---------------------------------------------------------------------
        // 5️⃣ Stop Shooter
        // ---------------------------------------------------------------------
        LSX.setVelocity(0);
        RSX.setVelocity(0);

        // ---------------------------------------------------------------------
        // 6️⃣ STOP EVERYTHING
        // ---------------------------------------------------------------------
        stopAll();
    }


    // =================== Helper Methods ===================

    private void drive(double lf, double lb, double rf, double rb, long timeMs) {
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

    private void stopAll() {
        LF.setPower(0);
        LB.setPower(0);
        RF.setPower(0);
        RB.setPower(0);
        LSX.setVelocity(0);
        RSX.setVelocity(0);
        IntakeEx.setVelocity(0);
    }
}