package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "V1-Functions", group = "Competition")
public class AutoV1 extends LinearOpMode {

    // ---------------- CONFIG ----------------
    // Launcher
    private double LAUNCHER_RPM = -500;            
    private long FIRST_LAUNCHER_SPINUP_MS = 5000;  
    private long SUBSEQUENT_LAUNCHER_SPINUP_MS = 500;

    // Flaps
    private double FLAP_UP_POS = 0.5;
    private double FLAP_DOWN_POS = 0.0;
    private long FLAP_UP_DURATION_MS = 300;

    // Intake
    private double INTAKE_POWER = 1.0;
    private long INTAKE_STEP4_MS = 2000;    
    private long INTAKE_STEP7_MS = 3000;    

    // Drive
    private double DRIVE_MAX_RPM = 6000;
    private double WHEEL_DIAMETER_IN = 4.0;
    private double INITIAL_BACKUP_IN = 12.0;  
    private double FINAL_BACKUP_IN = 84.0;    
    private double BACKUP_SPEED_FRACTION = 1.0; 
    // ---------------------------------------

    private DcMotorEx frontLeft, frontRight, backLeft, backRight;
    private DcMotorEx launcherLeft, launcherRight;
    private DcMotor intakeMotor;
    private Servo liftLeft, liftRight;

    private static final double TICKS_PER_REV = 28.0;
    private double DRIVE_TICKS_PER_SEC;

    private boolean firstLauncherSpinupDone = false;

    @Override
    public void runOpMode() {

        // Initialize hardware
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");
        launcherLeft = hardwareMap.get(DcMotorEx.class, "launcherLeft");
        launcherRight = hardwareMap.get(DcMotorEx.class, "launcherRight");
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        liftLeft = hardwareMap.get(Servo.class, "liftServoLeft");
        liftRight = hardwareMap.get(Servo.class, "liftServoRight");

        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        launcherRight.setDirection(DcMotorSimple.Direction.REVERSE);

        DRIVE_TICKS_PER_SEC = (DRIVE_MAX_RPM / 60.0) * TICKS_PER_REV;

        waitForStart();

        if (opModeIsActive()) {

            // Step 1: small reverse
            stopLauncher();
            driveDistance(-INITIAL_BACKUP_IN);

            // Step 2: launcher spin-up
            startLauncher();

            // Step 3: lift flaps briefly
            liftFlapAutoDown();

            // Step 4: intake for ball loading
            intakeMotor.setPower(INTAKE_POWER);
            sleep(INTAKE_STEP4_MS);
            intakeMotor.setPower(0);

            // Step 6: lift flaps briefly again
            liftFlapAutoDown();

            // Step 7: intake before final launch
            intakeMotor.setPower(INTAKE_POWER);
            sleep(INTAKE_STEP7_MS);
            intakeMotor.setPower(0);

            // Step 7c: shoot and stop launcher
            liftFlapAutoDown();
            stopLauncher();

            // Step 8: final reverse
            stopLauncher();
            driveDistance(-FINAL_BACKUP_IN);
        }
    }

    // ---------------- UTILITY ----------------
    private void startLauncher() {
        setLauncherPower(LAUNCHER_RPM);
        if (!firstLauncherSpinupDone) {
            sleep(FIRST_LAUNCHER_SPINUP_MS);
            firstLauncherSpinupDone = true;
        } else {
            sleep(SUBSEQUENT_LAUNCHER_SPINUP_MS);
        }
    }

    private void stopLauncher() {
        setLauncherPower(0);
    }

    private void setLauncherPower(double rpm) {
        double velocity = (rpm / 60.0) * TICKS_PER_REV;
        launcherLeft.setVelocity(velocity);
        launcherRight.setVelocity(velocity);
    }

    private void setFlap(double pos) {
        liftLeft.setPosition(pos);
        liftRight.setPosition(1.0 - pos);
    }

    private void liftFlapAutoDown() {
        setFlap(FLAP_UP_POS);
        sleep(FLAP_UP_DURATION_MS);
        setFlap(FLAP_DOWN_POS);
    }

    private void driveDistance(double inches) {
        double circumference = Math.PI * WHEEL_DIAMETER_IN;
        int ticks = (int)((inches / circumference) * TICKS_PER_REV);

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setTargetPosition(ticks);
        frontRight.setTargetPosition(ticks);
        backLeft.setTargetPosition(ticks);
        backRight.setTargetPosition(ticks);

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        double velocity = DRIVE_TICKS_PER_SEC * BACKUP_SPEED_FRACTION;
        frontLeft.setVelocity(velocity);
        frontRight.setVelocity(velocity);
        backLeft.setVelocity(velocity);
        backRight.setVelocity(velocity);

        while (frontLeft.isBusy() || frontRight.isBusy() ||
               backLeft.isBusy() || backRight.isBusy()) { }

        frontLeft.setVelocity(0);
        frontRight.setVelocity(0);
        backLeft.setVelocity(0);
        backRight.setVelocity(0);

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
