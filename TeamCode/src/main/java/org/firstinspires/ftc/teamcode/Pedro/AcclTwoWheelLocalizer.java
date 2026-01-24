package org.firstinspires.ftc.teamcode.Pedro;

import com.pedropathing.ftc.localization.CustomIMU;
import com.pedropathing.ftc.localization.constants.TwoWheelConstants;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.localization.Localizer;
import com.pedropathing.math.Matrix;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.math.Vector;
import com.pedropathing.util.NanoTimer;

/**
 * Full Enhanced TwoWheelLocalizer.
 * Accounts for Acceleration using the kinematic refinement: s = vt + 0.5at^2.
 */
public class AcclTwoWheelLocalizer implements Localizer {
    private final CustomIMU imu;
    private Pose startPose;
    private Pose displacementPose;
    private Pose currentVelocity;
    private Pose previousVelocity;
    private Pose currentAcceleration;

    private Matrix prevRotationMatrix;
    private final NanoTimer timer;
    private long deltaTimeNano;

    private final Encoder forwardEncoder;
    private final Encoder strafeEncoder;
    private final double strafePodX;
    private final double forwardPodY;
    private double previousIMUOrientation;
    private double totalHeading;

    public static double FORWARD_TICKS_TO_INCHES;
    public static double STRAFE_TICKS_TO_INCHES;

    public AcclTwoWheelLocalizer(HardwareMap map, TwoWheelConstants constants) {
        this(map, constants, new Pose());
    }

    public AcclTwoWheelLocalizer(HardwareMap map, TwoWheelConstants constants, Pose setStartPose) {
        FORWARD_TICKS_TO_INCHES = constants.forwardTicksToInches;
        STRAFE_TICKS_TO_INCHES = constants.strafeTicksToInches;
        imu = constants.imu;
        strafePodX = constants.strafePodX;
        forwardPodY = constants.forwardPodY;

        imu.initialize(map, constants.IMU_HardwareMapName, constants.IMU_Orientation);

        forwardEncoder = new Encoder(map.get(DcMotorEx.class, constants.forwardEncoder_HardwareMapName));
        strafeEncoder = new Encoder(map.get(DcMotorEx.class, constants.strafeEncoder_HardwareMapName));

        forwardEncoder.setDirection(constants.forwardEncoderDirection);
        strafeEncoder.setDirection(constants.strafeEncoderDirection);

        setStartPose(setStartPose);
        timer = new NanoTimer();
        deltaTimeNano = 1;
        displacementPose = new Pose();
        currentVelocity = new Pose();
        previousVelocity = new Pose();
        currentAcceleration = new Pose();

        previousIMUOrientation = MathFunctions.normalizeAngle(imu.getHeading());
        totalHeading = previousIMUOrientation;
    }

    @Override
    public void update() {
        deltaTimeNano = timer.getElapsedTime();
        timer.resetTimer();
        double seconds = deltaTimeNano / 1e9;

        if (seconds <= 0) return;

        updateEncoders();

        // 1. Calculate raw robot-relative deltas
        Matrix robotDeltas = getRobotDeltas();
        double dx = robotDeltas.get(0, 0);
        double dy = robotDeltas.get(1, 0);
        double dtheta = robotDeltas.get(2, 0);

        // 2. Calculate acceleration-refined deltas (s = v0t + 0.5at^2)
        Pose vNowLocal = new Pose(dx / seconds, dy / seconds, dtheta / seconds);
        currentAcceleration = new Pose(
                (vNowLocal.getX() - previousVelocity.getX()) / seconds,
                (vNowLocal.getY() - previousVelocity.getY()) / seconds,
                (vNowLocal.getHeading() - previousVelocity.getHeading()) / seconds
        );

        double refinedDX = dx + (0.5 * currentAcceleration.getX() * Math.pow(seconds, 2));
        double refinedDY = dy + (0.5 * currentAcceleration.getY() * Math.pow(seconds, 2));

        Matrix refinedDeltas = new Matrix(3, 1);
        refinedDeltas.set(0, 0, refinedDX);
        refinedDeltas.set(1, 0, refinedDY);
        refinedDeltas.set(2, 0, dtheta);

        // 3. Pose Exponential / Constant Curvature Kinematics
        setPrevRotationMatrix(getPose().getHeading());
        Matrix transformation = new Matrix(3, 3);
        if (Math.abs(dtheta) < 0.001) {
            transformation.set(0, 0, 1.0 - (Math.pow(dtheta, 2) / 6.0));
            transformation.set(0, 1, -dtheta / 2.0);
            transformation.set(1, 0, dtheta / 2.0);
            transformation.set(1, 1, 1.0 - (Math.pow(dtheta, 2) / 6.0));
        } else {
            transformation.set(0, 0, Math.sin(dtheta) / dtheta);
            transformation.set(0, 1, (Math.cos(dtheta) - 1.0) / dtheta);
            transformation.set(1, 0, (1.0 - Math.cos(dtheta)) / dtheta);
            transformation.set(1, 1, Math.sin(dtheta) / dtheta);
        }
        transformation.set(2, 2, 1.0);

        // 4. Transform to Global Space
        Matrix globalDeltas = Matrix.multiply(Matrix.multiply(prevRotationMatrix, transformation), refinedDeltas);

        // 5. Update Persistent States
        displacementPose = displacementPose.plus(new Pose(globalDeltas.get(0, 0), globalDeltas.get(1, 0), globalDeltas.get(2, 0)));
        previousVelocity = vNowLocal;
        currentVelocity = new Pose(globalDeltas.get(0, 0) / seconds, globalDeltas.get(1, 0) / seconds, globalDeltas.get(2, 0) / seconds);
        totalHeading += globalDeltas.get(2, 0);
    }

    public Matrix getRobotDeltas() {
        double currentIMUOrientation = MathFunctions.normalizeAngle(imu.getHeading());
        double deltaTheta = MathFunctions.getSmallestAngleDifference(currentIMUOrientation, previousIMUOrientation);
        previousIMUOrientation = currentIMUOrientation;

        double forwardDeltaInches = forwardEncoder.getDeltaPosition() * FORWARD_TICKS_TO_INCHES;
        double strafeDeltaInches = strafeEncoder.getDeltaPosition() * STRAFE_TICKS_TO_INCHES;

        // Offset correction for pod placement relative to center
        double robotDeltaX = forwardDeltaInches - (deltaTheta * forwardPodY);
        double robotDeltaY = strafeDeltaInches - (deltaTheta * strafePodX);

        Matrix m = new Matrix(3, 1);
        m.set(0, 0, robotDeltaX);
        m.set(1, 0, robotDeltaY);
        m.set(2, 0, deltaTheta);
        return m;
    }

    public void setPrevRotationMatrix(double heading) {
        prevRotationMatrix = new Matrix(3, 3);
        prevRotationMatrix.set(0, 0, Math.cos(heading));
        prevRotationMatrix.set(0, 1, -Math.sin(heading));
        prevRotationMatrix.set(1, 0, Math.sin(heading));
        prevRotationMatrix.set(1, 1, Math.cos(heading));
        prevRotationMatrix.set(2, 2, 1.0);
    }

    public void updateEncoders() {
        forwardEncoder.update();
        strafeEncoder.update();
    }

    @Override
    public Pose getPose() {
        return startPose.plus(displacementPose);
    }

    @Override
    public void setPose(Pose setPose) {
        startPose = setPose;
        displacementPose = new Pose();
    }

    @Override
    public Pose getVelocity() {
        return currentVelocity;
    }

    @Override
    public Vector getVelocityVector() {
        return new Vector(currentVelocity.getX(), currentVelocity.getY());
    }

    public Pose getAcceleration() {
        return currentAcceleration;
    }

    @Override
    public double getTotalHeading() {
        return totalHeading;
    }

    @Override
    public void setStartPose(Pose setStartPose) {
        this.startPose = setStartPose;
    }

    @Override
    public void resetIMU() {
        imu.resetYaw();
        previousIMUOrientation = 0;
    }

    /**
     * This returns the multiplier applied to forward movement measurement to convert from encoder
     * ticks to inches. This is found empirically through a tuner.
     *
     * @return returns the forward ticks to inches multiplier
     */
    public double getForwardMultiplier() {
        return FORWARD_TICKS_TO_INCHES;
    }

    /**
     * This returns the multiplier applied to lateral/strafe movement measurement to convert from
     * encoder ticks to inches. This is found empirically through a tuner.
     *
     * @return returns the lateral/strafe ticks to inches multiplier
     */
    public double getLateralMultiplier() {
        return STRAFE_TICKS_TO_INCHES;
    }

    /**
     * This returns the multiplier applied to turning movement measurement to convert from encoder
     * ticks to radians. This is found empirically through a tuner.
     *
     * @return returns the turning ticks to radians multiplier
     */
    public double getTurningMultiplier() {
        return 1;
    }


    /**
     * This is returns the IMU.
     *
     * @return returns the IMU
     */
    @Override
    public double getIMUHeading() {
        return imu.getHeading();
    }

    /**
     * This returns whether if any component of robot's position is NaN.
     *
     * @return returns whether the robot's position is NaN
     */
    public boolean isNAN() {
        return Double.isNaN(getPose().getX()) || Double.isNaN(getPose().getY()) || Double.isNaN(getPose().getHeading());
    }
}