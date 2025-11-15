package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "V2-EncoderBackUp", group = "Competition")
public class TeleOpV2 extends LinearOpMode {

    // Hardware
    private DcMotorEx frontLeft, frontRight, backLeft, backRight;
    private DcMotor intakeMotor;
    private DcMotorEx launcherLeft, launcherRight;
    private Servo liftLeft, liftRight;

    // Configurable Variables
    private double HIGH_RPM = -500;
    private double LOW_RPM = -300;
    private double FLAP_UP_POS = 0.5;
    private double FLAP_DOWN_POS = 0.0;
    private double DRIVE_DEADZONE = 0.05;
    private double DRIVE_MAX_RPM = 6000;
    private double WHEEL_DIAMETER = 4.0; // in inches
    private double BACKUP_DISTANCE = 12.0; // in inches, distance to move backward

    // Internal State
    private double targetRPM = 0;
    private boolean intakeToggle = false;
    private boolean lastIntakeToggle = false;
    private boolean flapUp = false;
    private boolean lastFlapToggle = false;
    private long flapTimer = 0;
    private boolean isBackingUp = false;
    private int backupTargetTicks = 0;

    private static final double TICKS_PER_REV = 28.0;
    private double DRIVE_TICKS_PER_SEC;

    @Override
    public void runOpMode() {
        initHardware();
        DRIVE_TICKS_PER_SEC = (DRIVE_MAX_RPM / 60.0) * TICKS_PER_REV;
        waitForStart();

        while (opModeIsActive()) {
            handleBackup();
            if (!isBackingUp) {
                mecanumDrive();
            }
            controlIntake();
            controlFlaps();
            updateLauncherVelocity();
            updateTelemetry();
        }
    }

    private void initHardware() {
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");

        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");

        launcherLeft = hardwareMap.get(DcMotorEx.class, "launcherLeft");
        launcherRight = hardwareMap.get(DcMotorEx.class, "launcherRight");

        liftLeft = hardwareMap.get(Servo.class, "liftServoLeft");
        liftRight = hardwareMap.get(Servo.class, "liftServoRight");

        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        launcherRight.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcherLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcherRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private void mecanumDrive() {
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = -gamepad1.right_stick_x;

        if (Math.abs(y) < DRIVE_DEADZONE) y = 0;
        if (Math.abs(x) < DRIVE_DEADZONE) x = 0;
        if (Math.abs(rx) < DRIVE_DEADZONE) rx = 0;

        double fl = y + x + rx;
        double fr = y - x - rx;
        double bl = y - x + rx;
        double br = y + x - rx;

        double max = Math.max(Math.abs(fl), Math.max(Math.abs(fr), Math.max(Math.abs(bl), Math.abs(br))));
        if (max > 1.0) {
            fl /= max;
            fr /= max;
            bl /= max;
            br /= max;
        }

        frontLeft.setVelocity(fl * DRIVE_TICKS_PER_SEC);
        frontRight.setVelocity(fr * DRIVE_TICKS_PER_SEC);
        backLeft.setVelocity(bl * DRIVE_TICKS_PER_SEC);
        backRight.setVelocity(br * DRIVE_TICKS_PER_SEC);
    }

    private void handleBackup() {
        if (gamepad1.x && !isBackingUp) {
            double wheelCircumference = Math.PI * WHEEL_DIAMETER;
            double rotations = BACKUP_DISTANCE / wheelCircumference;
            backupTargetTicks = (int)(rotations * TICKS_PER_REV);

            frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            frontLeft.setTargetPosition(-backupTargetTicks);
            frontRight.setTargetPosition(-backupTargetTicks);
            backLeft.setTargetPosition(-backupTargetTicks);
            backRight.setTargetPosition(-backupTargetTicks);

            frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            frontLeft.setVelocity(DRIVE_TICKS_PER_SEC);
            frontRight.setVelocity(DRIVE_TICKS_PER_SEC);
            backLeft.setVelocity(DRIVE_TICKS_PER_SEC);
            backRight.setVelocity(DRIVE_TICKS_PER_SEC);

            isBackingUp = true;
        }

        if (isBackingUp) {
            if (!frontLeft.isBusy() && !frontRight.isBusy() && !backLeft.isBusy() && !backRight.isBusy()) {
                frontLeft.setVelocity(0);
                frontRight.setVelocity(0);
                backLeft.setVelocity(0);
                backRight.setVelocity(0);

                frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                isBackingUp = false;
            }
        }
    }

    private void updateLauncherVelocity() {
        if (gamepad1.right_trigger > 0.5) targetRPM = HIGH_RPM;
        else if (gamepad1.left_trigger > 0.5) targetRPM = LOW_RPM;
        else targetRPM = 0;

        double velocity = (targetRPM / 60.0) * TICKS_PER_REV;
        launcherLeft.setVelocity(velocity);
        launcherRight.setVelocity(velocity);
    }

    private void controlIntake() {
        if (gamepad1.right_bumper && !lastIntakeToggle) intakeToggle = !intakeToggle;
        lastIntakeToggle = gamepad1.right_bumper;

        double intakePower = 0.0;
        if (gamepad1.a) intakePower = 1.0;
        else if (gamepad1.b) intakePower = -1.0;
        else if (intakeToggle) intakePower = 1.0;

        intakeMotor.setPower(intakePower);
    }

    private void controlFlaps() {
        if (gamepad1.left_bumper && !lastFlapToggle) {
            flapUp = !flapUp;
            if (flapUp) flapTimer = System.currentTimeMillis();
        }
        lastFlapToggle = gamepad1.left_bumper;

        if (flapUp && System.currentTimeMillis() - flapTimer > 300) flapUp = false;

        liftLeft.setPosition(flapUp ? FLAP_UP_POS : FLAP_DOWN_POS);
        liftRight.setPosition(flapUp ? 1 - FLAP_UP_POS : 1 - FLAP_DOWN_POS);
    }

    private void updateTelemetry() {
        telemetry.addData("Launcher Target RPM", targetRPM);
        telemetry.addData("Left RPM", launcherLeft.getVelocity() / TICKS_PER_REV * 60.0);
        telemetry.addData("Right RPM", launcherRight.getVelocity() / TICKS_PER_REV * 60.0);
        telemetry.addData("Backing Up?", isBackingUp);
        telemetry.update();
    }
}
