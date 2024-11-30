package org.firstinspires.ftc.teamcode;

import static com.acmerobotics.roadrunner.Actions.now;

import static org.firstinspires.ftc.teamcode.StandardSetupOpMode.AUTO_MOVE_POWER;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.HolonomicController;
import com.acmerobotics.roadrunner.MecanumKinematics;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.MotorFeedforward;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.PoseVelocity2dDual;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.ProfileParams;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.TimeTrajectory;
import com.acmerobotics.roadrunner.TimeTurn;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TrajectoryBuilderParams;
import com.acmerobotics.roadrunner.TurnConstraints;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.DownsampledWriter;
import com.acmerobotics.roadrunner.ftc.FlightRecorder;
import com.acmerobotics.roadrunner.ftc.LynxFirmware;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.messages.DriveCommandMessage;
import org.firstinspires.ftc.teamcode.messages.MecanumCommandMessage;
import org.firstinspires.ftc.teamcode.messages.PoseMessage;

import java.lang.Math;
import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;

@Config
public final class MecanumDrive extends BodyPart{
    public static class Params {
        // Mecanum kinematics parameters
        public double trackWidthInches = 16.55;
        public double wheelSlipMultiplier = 1.72;

        // feedforward parameters (in inches)
        public double kS = 0.869786849089929;
        public double kV = 0.15;
        public double kA = 0.034;

        // path profile parameters (in inches)
        public double maxWheelVel = 50;
        public double minProfileAccel = -30;
        public double maxProfileAccel = 50;

        // turn profile parameters (in radians)
        public double maxAngVel = Math.PI; // shared with path
        public double maxAngAccel = Math.PI;

        // path controller gains
        public double axialGain = 0.0;
        public double lateralGain = 0.0;
        public double headingGain = 1.0; // shared with turn

        public double axialVelGain = 0.0;
        public double lateralVelGain = 0.0;
        public double headingVelGain = 0.0; // shared with turn
    }

    public static Params PARAMS = new Params();

    public final MecanumKinematics kinematics = new MecanumKinematics(
            PARAMS.trackWidthInches, PARAMS.wheelSlipMultiplier);

    public final TurnConstraints defaultTurnConstraints = new TurnConstraints(
            PARAMS.maxAngVel, -PARAMS.maxAngAccel, PARAMS.maxAngAccel);
    public final VelConstraint defaultVelConstraint =
            new MinVelConstraint(Arrays.asList(
                    kinematics.new WheelVelConstraint(PARAMS.maxWheelVel),
                    new AngularVelConstraint(PARAMS.maxAngVel)
            ));
    public final AccelConstraint defaultAccelConstraint =
            new ProfileAccelConstraint(PARAMS.minProfileAccel, PARAMS.maxProfileAccel);

    public final DcMotorEx leftFront, leftBack, rightBack, rightFront;

    public final VoltageSensor voltageSensor;

    public final Localizer localizer;
    private Pose2d pose;
    private final Gamepad gamepad;
    private boolean ignoreGamepad;

    private Telemetry telemetry;

    private final LinkedList<Pose2d> poseHistory = new LinkedList<>();

    private final DownsampledWriter estimatedPoseWriter = new DownsampledWriter("ESTIMATED_POSE", 50_000_000);
    private final DownsampledWriter targetPoseWriter = new DownsampledWriter("TARGET_POSE", 50_000_000);
    private final DownsampledWriter driveCommandWriter = new DownsampledWriter("DRIVE_COMMAND", 50_000_000);
    private final DownsampledWriter mecanumCommandWriter = new DownsampledWriter("MECANUM_COMMAND", 50_000_000);

    public MecanumDrive(HardwareMap hardwareMap, Pose2d pose, Gamepad gamepad, Telemetry telemetry) {
        this.pose = pose;
        this.gamepad = gamepad;
        this.ignoreGamepad = false;
        this.telemetry = telemetry;

        LynxFirmware.throwIfModulesAreOutdated(hardwareMap);

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // TODO: make sure your config has motors with these names (or change them)
        //   see https://ftc-docs.firstinspires.org/en/latest/hardware_and_software_configuration/configuring/index.html
        leftFront = hardwareMap.get(DcMotorEx.class, "frontLeftDrive");
        leftBack = hardwareMap.get(DcMotorEx.class, "rearLeftDrive");
        rightBack = hardwareMap.get(DcMotorEx.class, "rearRightDrive");
        rightFront = hardwareMap.get(DcMotorEx.class, "frontRightDrive");

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // TODO: reverse motor directions if needed
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        voltageSensor = hardwareMap.voltageSensor.iterator().next();
        localizer = new GoBildaPinpointLocalizer(hardwareMap);
        ((GoBildaPinpointLocalizer)localizer).reset();

        FlightRecorder.write("MECANUM_PARAMS", PARAMS);
    }

    /**
     * Method adds important things to telemetry
     * @param telemetry place to store telemetry
     */
    public void debugTelemetry(Telemetry telemetry)
    {
        if(telemetry == null) return;

        //GoBildaPinpointLocalizer plocalizer = (GoBildaPinpointLocalizer)localizer;
        //Pose2D ftc_pose = plocalizer.get_ftc_pose();
        //telemetry.addData("Localizer x (in)", "%f", ftc_pose.getX(DistanceUnit.INCH));
        //telemetry.addData("Localizer y (in)", "%f", ftc_pose.getY(DistanceUnit.INCH));
        //telemetry.addData("Localizer heading (deg)", "%f", ftc_pose.getHeading(AngleUnit.DEGREES));

        telemetry.addData("Pose x (in)", "%f", pose.position.x);
        telemetry.addData("Pose y (in)", "%f", pose.position.y);
        telemetry.addData("Pose heading (deg)", "%f", Math.toDegrees(pose.heading.toDouble()));
    }

    public void setDrivePowers(PoseVelocity2d powers) {
        MecanumKinematics.WheelVelocities<Time> wheelVels = new MecanumKinematics(1).inverse(PoseVelocity2dDual.constant(powers, 1));

        double maxPowerMag = 1;
        for (DualNum<Time> power : wheelVels.all()) {
            maxPowerMag = Math.max(maxPowerMag, power.value());
        }

        leftFront.setPower(wheelVels.leftFront.get(0) / maxPowerMag);
        leftBack.setPower(wheelVels.leftBack.get(0) / maxPowerMag);
        rightBack.setPower(wheelVels.rightBack.get(0) / maxPowerMag);
        rightFront.setPower(wheelVels.rightFront.get(0) / maxPowerMag);
    }

    public final class FollowTrajectoryAction implements Action {
        public final TimeTrajectory timeTrajectory;
        private double beginTs = -1;

        private final double[] xPoints, yPoints;

        public FollowTrajectoryAction(TimeTrajectory t) {
            timeTrajectory = t;

            List<Double> disps = com.acmerobotics.roadrunner.Math.range(
                    0, t.path.length(),
                    Math.max(2, (int) Math.ceil(t.path.length() / 2)));
            xPoints = new double[disps.size()];
            yPoints = new double[disps.size()];
            for (int i = 0; i < disps.size(); i++) {
                Pose2d p = t.path.get(disps.get(i), 1).value();
                xPoints[i] = p.position.x;
                yPoints[i] = p.position.y;
            }
        }

        @Override
        public boolean run(@NonNull TelemetryPacket p) {
            double t;
            if (beginTs < 0) {
                beginTs = now();
                t = 0;
            } else {
                t = now() - beginTs;
            }

            if (t >= timeTrajectory.duration) {
                leftFront.setPower(0);
                leftBack.setPower(0);
                rightBack.setPower(0);
                rightFront.setPower(0);

                return false;
            }

            Pose2dDual<Time> txWorldTarget = timeTrajectory.get(t);
            targetPoseWriter.write(new PoseMessage(txWorldTarget.value()));

            PoseVelocity2d robotVelRobot = updatePoseEstimate();

            PoseVelocity2dDual<Time> command = new HolonomicController(
                    PARAMS.axialGain, PARAMS.lateralGain, PARAMS.headingGain,
                    PARAMS.axialVelGain, PARAMS.lateralVelGain, PARAMS.headingVelGain
            )
                    .compute(txWorldTarget, pose, robotVelRobot);
            driveCommandWriter.write(new DriveCommandMessage(command));

            MecanumKinematics.WheelVelocities<Time> wheelVels = kinematics.inverse(command);
            double voltage = voltageSensor.getVoltage();

            final MotorFeedforward feedforward = new MotorFeedforward(
                    PARAMS.kS, PARAMS.kV, PARAMS.kA);
            double leftFrontPower = feedforward.compute(wheelVels.leftFront) / voltage;
            double leftBackPower = feedforward.compute(wheelVels.leftBack) / voltage;
            double rightBackPower = feedforward.compute(wheelVels.rightBack) / voltage;
            double rightFrontPower = feedforward.compute(wheelVels.rightFront) / voltage;
            mecanumCommandWriter.write(new MecanumCommandMessage(
                    voltage, leftFrontPower, leftBackPower, rightBackPower, rightFrontPower
            ));

            leftFront.setPower(leftFrontPower);
            leftBack.setPower(leftBackPower);
            rightBack.setPower(rightBackPower);
            rightFront.setPower(rightFrontPower);

            p.put("x", pose.position.x);
            p.put("y", pose.position.y);
            p.put("heading (deg)", Math.toDegrees(pose.heading.toDouble()));

            Pose2d error = txWorldTarget.value().minusExp(pose);
            p.put("xError", error.position.x);
            p.put("yError", error.position.y);
            p.put("headingError (deg)", Math.toDegrees(error.heading.toDouble()));

            // only draw when active; only one drive action should be active at a time
            Canvas c = p.fieldOverlay();
            drawPoseHistory(c);

            c.setStroke("#4CAF50");
            Drawing.drawRobot(c, txWorldTarget.value());

            c.setStroke("#3F51B5");
            Drawing.drawRobot(c, pose);

            c.setStroke("#4CAF50FF");
            c.setStrokeWidth(1);
            c.strokePolyline(xPoints, yPoints);

            return true;
        }

        @Override
        public void preview(Canvas c) {
            c.setStroke("#4CAF507A");
            c.setStrokeWidth(1);
            c.strokePolyline(xPoints, yPoints);
        }
    }

    public final class TurnAction implements Action {
        private final TimeTurn turn;

        private double beginTs = -1;

        public TurnAction(TimeTurn turn) {
            this.turn = turn;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket p) {
            double t;
            if (beginTs < 0) {
                beginTs = now();
                t = 0;
            } else {
                t = now() - beginTs;
            }

            if (t >= turn.duration) {
                leftFront.setPower(0);
                leftBack.setPower(0);
                rightBack.setPower(0);
                rightFront.setPower(0);

                return false;
            }

            Pose2dDual<Time> txWorldTarget = turn.get(t);
            targetPoseWriter.write(new PoseMessage(txWorldTarget.value()));

            PoseVelocity2d robotVelRobot = updatePoseEstimate();

            PoseVelocity2dDual<Time> command = new HolonomicController(
                    PARAMS.axialGain, PARAMS.lateralGain, PARAMS.headingGain,
                    PARAMS.axialVelGain, PARAMS.lateralVelGain, PARAMS.headingVelGain
            )
                    .compute(txWorldTarget, pose, robotVelRobot);
            driveCommandWriter.write(new DriveCommandMessage(command));

            MecanumKinematics.WheelVelocities<Time> wheelVels = kinematics.inverse(command);
            double voltage = voltageSensor.getVoltage();
            final MotorFeedforward feedforward = new MotorFeedforward(
                    PARAMS.kS, PARAMS.kV, PARAMS.kA);
            double leftFrontPower = feedforward.compute(wheelVels.leftFront) / voltage;
            double leftBackPower = feedforward.compute(wheelVels.leftBack) / voltage;
            double rightBackPower = feedforward.compute(wheelVels.rightBack) / voltage;
            double rightFrontPower = feedforward.compute(wheelVels.rightFront) / voltage;
            mecanumCommandWriter.write(new MecanumCommandMessage(
                    voltage, leftFrontPower, leftBackPower, rightBackPower, rightFrontPower
            ));

            leftFront.setPower(feedforward.compute(wheelVels.leftFront) / voltage);
            leftBack.setPower(feedforward.compute(wheelVels.leftBack) / voltage);
            rightBack.setPower(feedforward.compute(wheelVels.rightBack) / voltage);
            rightFront.setPower(feedforward.compute(wheelVels.rightFront) / voltage);

            Canvas c = p.fieldOverlay();
            drawPoseHistory(c);

            c.setStroke("#4CAF50");
            Drawing.drawRobot(c, txWorldTarget.value());

            c.setStroke("#3F51B5");
            Drawing.drawRobot(c, pose);

            c.setStroke("#7C4DFFFF");
            c.fillCircle(turn.beginPose.position.x, turn.beginPose.position.y, 2);

            return true;
        }

        @Override
        public void preview(Canvas c) {
            c.setStroke("#7C4DFF7A");
            c.fillCircle(turn.beginPose.position.x, turn.beginPose.position.y, 2);
        }
    }

    public PoseVelocity2d updatePoseEstimate() {
        GoBildaPinpointLocalizer local = (GoBildaPinpointLocalizer)localizer;
        pose = local.get_pose_estimate();

        poseHistory.add(pose);
        while (poseHistory.size() > 100) {
            poseHistory.removeFirst();
        }

        estimatedPoseWriter.write(new PoseMessage(pose));

        return local.get_velocity_estimate();
    }

    private void drawPoseHistory(Canvas c) {
        double[] xPoints = new double[poseHistory.size()];
        double[] yPoints = new double[poseHistory.size()];

        int i = 0;
        for (Pose2d t : poseHistory) {
            xPoints[i] = t.position.x;
            yPoints[i] = t.position.y;

            i++;
        }

        c.setStrokeWidth(1);
        c.setStroke("#3F51B5");
        c.strokePolyline(xPoints, yPoints);
    }

    public TrajectoryActionBuilder actionBuilder(Pose2d beginPose) {
        return new TrajectoryActionBuilder(
                TurnAction::new,
                FollowTrajectoryAction::new,
                new TrajectoryBuilderParams(
                        1e-6,
                        new ProfileParams(
                                0.25, 0.1, 1e-2
                        )
                ),
                beginPose, 0.0,
                defaultTurnConstraints,
                defaultVelConstraint, defaultAccelConstraint
        );
    }

    private Pose2d leftPose = null;
    private Pose2d rightPose = null;

    @Override
    public void run() {
        boolean oneCancel = false;
        while (!isInterrupted()  && gamepad != null && !ignoreGamepad) {
            if (gamepad.dpad_left) {
                if (!moveThread.isAlive()) telemetry.addLine("move left"); telemetry.update(); moveLeft(10.0);
            } else if (gamepad.dpad_right) {
                if (!moveThread.isAlive()) moveLeft(-10.0);
            } else if (gamepad.dpad_up) {
                if (!moveThread.isAlive()) moveForward(10.0);
            } else if (gamepad.dpad_down) {
                if (!moveThread.isAlive()) moveForward(-10.0);
            } else if (gamepad.x) {
                if (!moveThread.isAlive()) rotate(90.0);
            } else if (gamepad.b) {
                if (!moveThread.isAlive()) rotate(-90.0);
            } else if (gamepad.left_bumper) {
                leftPose = getPose();
            } else if (gamepad.right_bumper) {
                rightPose = getPose();
            } else if (gamepad.left_trigger > 0.8) {
                if (!moveThread.isAlive()) moveToPose(AUTO_MOVE_POWER, leftPose);
            } else if (gamepad.right_trigger > 0.8) {
                if (!moveThread.isAlive()) moveToPose(AUTO_MOVE_POWER, rightPose);
            } else if(Math.abs(gamepad.left_stick_x) > 0.01 ||
                    Math.abs(gamepad.left_stick_y) > 0.01 ||
                    Math.abs(gamepad.right_stick_x) > 0.01)
            {
                // Interrupt old move thread
                if(moveThread.isAlive())
                    moveThread.interrupt();

                // See if the user would like to slow down
                double speedFactor = gamepad.y ? 0.25 : gamepad.a ? 0.5 : 1.0;

                // Map joystick values to drive powers
                setDrivePowers(new PoseVelocity2d(
                        new Vector2d(
                                -gamepad.left_stick_y * speedFactor,
                                -gamepad.left_stick_x * speedFactor
                        ),
                        -gamepad.right_stick_x * speedFactor
                ));
                oneCancel = true;
            } else if(oneCancel){
                setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
                oneCancel = false;
                updatePoseEstimate();
            }
        }
    }

    /**
     * Quick rotate
     * @param degrees amount to turn in degrees
     */
    public void rotate(double degrees) {
        Pose2d currentPose = getPose();
        Pose2d newPose = new Pose2d(currentPose.position, currentPose.heading.plus(Math.toRadians(degrees)));
        moveToPose(AUTO_MOVE_POWER, newPose);
    }

    /**
     * Amount to move forward
     * @param dist inches
     */
    public void moveForward(double dist) {
        Pose2d currentPose = getPose();
        double currentHeading = currentPose.heading.toDouble();
        Vector2d forwardUnit = new Vector2d(Math.cos(currentHeading), Math.sin(currentHeading));
        Vector2d forward = forwardUnit.times(dist);
        Pose2d newPose = new Pose2d(currentPose.position.plus(forward), currentPose.heading);
        moveToPose(AUTO_MOVE_POWER, newPose);
    }

    /**
     * Amount to move left
     * @param dist inches
     */
    public void moveLeft(double dist) {
        Pose2d currentPose = getPose();
        double currentHeading = currentPose.heading.toDouble();
        Vector2d leftUnit = new Vector2d(-Math.sin(currentHeading), Math.cos(currentHeading));
        Vector2d left = leftUnit.times(dist);
        Pose2d newPose = new Pose2d(currentPose.position.plus(left), currentPose.heading);
        moveToPose(AUTO_MOVE_POWER, newPose);
    }

    /**
     * Gives the actual current pose estimate of the robot
     * @return the current pose from the pinpoint computer
     */
    public Pose2d getPose(){
        return ((GoBildaPinpointLocalizer) localizer).get_pose_estimate();
    }

    /**
     * This method generates a move action that considers the current position.
     *
     * @param power maximum power to apply to motors
     * @param expectedPose The pose you would like to end with
     * @return The action you can run in your action list
     */
    public Action moveToAction(double power, Pose2d expectedPose, int direction, long timeout_ms)
    {
        return telemetryPacket -> {
            moveToPose(power, expectedPose, direction, timeout_ms);
            return false;
        };
    }
    public Action moveToAction(double power, Pose2d expectedPose, int direction)
    {
        return moveToAction(power, expectedPose, direction, DEFAULT_TIMEOUT_MS);
    }
    public Action moveToAction(double power, Pose2d expectedPose)
    {
        return moveToAction(power, expectedPose, DEFAULT_SPIN_DIRECTION, DEFAULT_TIMEOUT_MS);
    }

    // Tweak these variables if you have wheel slip
    // Ramp down wheel slip will cause overshoot that may need to be corrected
    // but increasing these values will slow the movement down and increase the
    // time it takes to locate our position.  Tweaking close enough will also change
    // run time and accuracy.
    private static final int DEFAULT_SPIN_DIRECTION = 0;        // Default, we don't care
    private static final long DEFAULT_TIMEOUT_MS = 5000;        // Give yourself 5 seconds
    private static final double RAMP_DOWN_FORWARD = 15.0;       // 15 inches from target, ramp down
    private static final double RAMP_DOWN_LEFT = 10.0;          // 10 inches left target, ramp down
    private static final double RAMP_DOWN_ANGLE = 60.0;         // 60 degrees from target, ramp down
    private static final double MIN_FORWARD_POWER = 0.15;       // Minimum power when not close enough
    private static final double MIN_LEFT_POWER = 0.2;           // Minimum power when not close enough
    private static final double MIN_ROTATION_POWER = 0.15;      // Minimum power when not close enough
    private static final double CLOSE_ENOUGH_POSITION = 0.5;    // Stop when this close
    private static final double CLOSE_ENOUGH_ANGLE = 1.0;       // Stop within this angle
    private static final double CLOSE_ENOUGH_SPIN = 45.0;       // Flip spin direction if further

    static
    {
        assert RAMP_DOWN_FORWARD != 0 : "RAMP_DOWN_FORWARD cannot be 0";
        assert RAMP_DOWN_LEFT != 0 : "RAMP_DOWN_LEFT cannot be 0";
        assert RAMP_DOWN_ANGLE != 0 : "RAMP_DOWN_ANGLE cannot be 0";
    }

    private static double minTowardZero(double a, double b)
    {
        return (Math.abs(a) < Math.abs(b)) ? a : b;
    }
    private static double maxFromZero(double a, double b)
    {
        return (Math.abs(a) > Math.abs(b)) ? a : b;
    }

    /**
     * Ths class removes the need for Roadrunner kinematics which are notoriously
     * hard to dial in.  This uses the fact that we have a very accurate pinpoint
     * computer constantly feeding us robot positions.  That allows us to drive
     * the robot like a human watching the positions and angles very closely.
     * We are sacrificing splines for straight paths but the advantage should be
     * accuracy and repeatability.
     */
    private class MoveThread extends Thread
    {
        private final double power;         // maximum power to apply + and - (capped to [-1.0,1.0]
        private final Pose2d endPose;       // final target pose (used for ramp down)
        private final int spinDirection;    // positive left, 0 don't care, negative right
        private final long maxTime_ms;      // Maximum time allowed to drive in milliseconds

        /**
         * A thread to move the robot, be careful when you call this
         * @param power maximum power allowed
         * @param endPose final pose of the robot
         * @param spinDirection direction you would prefer the robot to spin
         * @param maxTime_ms maximum number of milliseconds before we stop moving
         */
        public MoveThread(double power, Pose2d endPose, int spinDirection, long maxTime_ms){
            // Ensure the power doesn't exceed +/- 1.0
            this.power = minTowardZero(power, Math.signum(power));

            // Save the final target position and preferred spin direction
            this.endPose = endPose;
            this.spinDirection = spinDirection;
            this.maxTime_ms = Math.abs(maxTime_ms);
        }

        @Override
        public void run()
        {
            // Setup for the run getting the starting pose and the current time
            long start_ms = System.currentTimeMillis();
            long current_ms;
            long cnt = 0;

            // Loop until we are close enough or run out of time
            do
            {
                // Have we been interrupted
                if(isInterrupted()) break;

                // Get current position
                Pose2d currentPose = getPose();

                // Vector of travel
                Vector2d nextDeltaPos = endPose.position.minus(currentPose.position);
                double nextDeltaAng = Math.toDegrees(endPose.heading.toDouble() - currentPose.heading.toDouble());

                // Ensure the delta angle is less than 360
                nextDeltaAng = nextDeltaAng % 360.0;

                // If we don't care about spin direction (use shortest spin)
                if(spinDirection == 0) {
                    if (nextDeltaAng > 180.0)
                        nextDeltaAng -= 360.0;
                    if (nextDeltaAng < -180.0)
                        nextDeltaAng += 360.0;
                }
                // Make sure we spin left
                else if (spinDirection > 0)
                {
                    if(nextDeltaAng < -CLOSE_ENOUGH_SPIN)
                        nextDeltaAng += 360.0;
                }
                // Make sure we spin right
                else {
                    if(nextDeltaAng > CLOSE_ENOUGH_SPIN)
                        nextDeltaAng -= 360.0;
                }

                // Direction unit vector (along body of robot)
                double currentHeading = currentPose.heading.toDouble();
                Vector2d forwardUnit = new Vector2d(Math.cos(currentHeading), Math.sin(currentHeading));
                Vector2d leftUnit = new Vector2d(-Math.sin(currentHeading), Math.cos(currentHeading));

                // How far do we need to go along our heading
                double nextForward = nextDeltaPos.dot(forwardUnit);
                double nextLeft = nextDeltaPos.dot(leftUnit);

                // If we're close enough we are done
                boolean closeEnoughForward = Math.abs(nextForward) < CLOSE_ENOUGH_POSITION;
                boolean closeEnoughLeft = Math.abs(nextLeft) < CLOSE_ENOUGH_POSITION;
                boolean closeEnoughAngle = Math.abs(nextDeltaAng) < CLOSE_ENOUGH_ANGLE;
                if(closeEnoughForward && closeEnoughLeft && closeEnoughAngle)
                    break;

                // What powers do we need to get to our final destination
                double forwardPower = nextForward / RAMP_DOWN_FORWARD;
                double leftPower = nextLeft / RAMP_DOWN_LEFT;
                double rotationPower = nextDeltaAng / RAMP_DOWN_ANGLE;

                // Ensure we still have wiggle power if we're really close
                if(!closeEnoughForward){
                    forwardPower = maxFromZero(forwardPower, Math.signum(forwardPower) * MIN_FORWARD_POWER);
                }
                if(!closeEnoughLeft){
                    leftPower = maxFromZero(leftPower, Math.signum(leftPower) * MIN_LEFT_POWER);
                }
                if(!closeEnoughAngle) {
                    rotationPower = maxFromZero(rotationPower, Math.signum(rotationPower) * MIN_ROTATION_POWER);
                }

                // Cap the powers
                forwardPower = minTowardZero(forwardPower, power * Math.signum(forwardPower));
                leftPower = minTowardZero(leftPower, power * Math.signum(leftPower));
                rotationPower = minTowardZero(rotationPower, power * Math.signum(rotationPower));

                // Debug
                telemetry.addData("How Close Position", "%f", nextDeltaPos.norm());
                telemetry.addData("How Cose Angle", "%f", nextDeltaAng);
                telemetry.addData("Foward Power", "%f", forwardPower);
                telemetry.addData("Left Power", "%f", leftPower);
                telemetry.addData("Rotation Power", "%f", rotationPower);
                telemetry.addData("Count", ++cnt);
                telemetry.addData("getNumListeners", getNumListeners());
                telemetry.update();

                // Tell the motors
                setDrivePowers(new PoseVelocity2d(new Vector2d(forwardPower,leftPower),rotationPower));

                // Latest time (while loop checks if we timed out)
                current_ms = System.currentTimeMillis();
            }while(maxTime_ms > current_ms - start_ms);

            // Stop the drive motors and notify listeners that we have arrived
            setDrivePowers(new PoseVelocity2d(new Vector2d(0,0),0));

            // If we are interrupted then they will handle notify
            if(!isInterrupted())
              notifyOldestListener();
        }
    }

    private Thread moveThread = new Thread();

    @Override
    public int getCurrentPosition()
    {
        return 0;
    }

    @Override
    public void safeHold(int position)
    {
    }

    /**
     * A threaded method that moves the robot to the tasked pose.  This method takes a direct
     * line of sight path to the final pose and acts like a human driving the robot.  This is
     * meant to be very accurate and replace the road runner trajectory methods.
     * @param power maximum power to apply to the motors [-1.0,1.0]
     * @param expectedPose final pose of the robot
     * @param spinDirection direction you would like to spin (positive is left, 0 is don't care, negative is right)
     * @param timeout_ms number of milliseconds before we cancel our motion
     */
    public void moveToPose(double power, Pose2d expectedPose, int spinDirection, long timeout_ms)
    {
        // Start a new thread that keeps setting drive powers until we hit our spot
        if(expectedPose != null) {
            // Cancel previous threads
            moveThread.interrupt();

            // Start the new thread
            moveThread = new MoveThread(power, expectedPose, spinDirection, timeout_ms);
            moveThread.start();
        }
    }

    /**
     * Overloaded method that doesn't care about turn direction and uses the standard timeout
     * @param power max power for motors
     * @param expectedPose final pose
     */
    public void moveToPose(double power, Pose2d expectedPose)
    {
        moveToPose(power, expectedPose, DEFAULT_SPIN_DIRECTION, DEFAULT_TIMEOUT_MS);
    }

    /**
     * Externally disable the gamepad
     * @param ignoreGamepad true will ignore gamepad, good for autonomous
     */
    public void setIgnoreGamepad(boolean ignoreGamepad)
    {
        this.ignoreGamepad = ignoreGamepad;
    }
}
