package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.command.SubsystemBase;
import org.firstinspires.ftc.teamcode.util.Alliance;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * SUBSYSTEM: Limelight 3A, built for BIOBUZZ HIVE CELL AprilTag CLUSTERS.
 *
 * LAST SEASON vs THIS SEASON
 *   DECODE:  one tag per goal  -> "find tag N, aim at it". The goal never moved.
 *   BIOBUZZ: four tags per CELL, two CELLS per HIVE, and the HIVE FLIPS when it
 *            tips. So every loop we:
 *              1. read every tag the camera sees
 *              2. GROUP them into their CELL cluster (any 1 of 4 tags is enough)
 *              3. average the tags -> one steadier aim angle, distance and roll
 *              4. keep only OUR alliance's cells
 *              5. keep only the UPRIGHT cell (|roll| < 90) — that's the scorable one
 *
 * This is the same idea as the FTC SDK 12 AprilTagClusterDetection, done by hand
 * because the Limelight reports tags one at a time.
 *
 * SAFE WITHOUT A CAMERA: if no "limelight" is in the Robot Configuration,
 * isConnected() is false and every query returns "nothing seen".
 */
public class VisionSubsystem extends SubsystemBase {

    /** One HIVE CELL, built from however many of its 4 tags are visible. */
    public static class HiveCell {
        public final String name;          // e.g. "RED SCORING"
        public final int tagsSeen;         // 1..4
        public final double percentFound;  // 25, 50, 75 or 100
        public final double txDeg;         // average horizontal angle, + = right of crosshair
        public final double tyDeg;         // average vertical angle
        public final double distanceIn;    // average floor distance, NaN if 3D pose unavailable
        public final double rollDeg;       // average roll, after ROLL_OFFSET_DEG

        HiveCell(String name, int tagsSeen, double txDeg, double tyDeg,
                 double distanceIn, double rollDeg) {
            this.name = name;
            this.tagsSeen = tagsSeen;
            this.percentFound = tagsSeen * 25.0;
            this.txDeg = txDeg;
            this.tyDeg = tyDeg;
            this.distanceIn = distanceIn;
            this.rollDeg = rollDeg;
        }

        /** Upright = opening faces up = we can score in it. */
        public boolean isUpright() {
            return Math.abs(rollDeg) < Constants.Vision.UPRIGHT_MAX_ROLL_DEG;
        }

        public boolean belongsTo(Alliance alliance) {
            return name.startsWith(alliance.cellPrefix);
        }

        @Override
        public String toString() {
            return String.format("%-13s %3.0f%%  %s  tx %5.1f  dist %s  roll %4.0f",
                    name, percentFound, isUpright() ? "UP  " : "down", txDeg,
                    Double.isNaN(distanceIn) ? "  -" : String.format("%3.0f\"", distanceIn), rollDeg);
        }
    }

    private final Limelight3A limelight; // null when not configured
    private List<HiveCell> cells = Collections.emptyList();
    private List<Integer> rawTagIds = Collections.emptyList();

    public VisionSubsystem(HardwareMap hardwareMap) {
        // tryGet returns null instead of crashing when the device isn't configured.
        limelight = hardwareMap.tryGet(Limelight3A.class, Constants.Hardware.LIMELIGHT);
        if (limelight != null) {
            limelight.setPollRateHz(100);
            limelight.pipelineSwitch(Constants.Vision.APRILTAG_PIPELINE);
            limelight.start();
        }
    }

    /** Every loop: read tags and rebuild the list of visible HIVE CELLS. */
    @Override
    public void periodic() {
        if (limelight == null) return;

        LLResult r = limelight.getLatestResult();
        boolean fresh = r != null && r.isValid()
                && r.getStaleness() < Constants.Vision.MAX_STALENESS_MS
                && r.getPipelineIndex() == Constants.Vision.APRILTAG_PIPELINE;
        if (!fresh) {
            cells = Collections.emptyList();
            rawTagIds = Collections.emptyList();
            return;
        }

        List<LLResultTypes.FiducialResult> tags = r.getFiducialResults();
        List<Integer> ids = new ArrayList<>();
        for (LLResultTypes.FiducialResult t : tags) ids.add(t.getFiducialId());
        rawTagIds = ids;
        cells = groupIntoCells(tags);
    }

    /** Step 2 + 3: sort tags into their CELL by ID, then average each group. */
    private static List<HiveCell> groupIntoCells(List<LLResultTypes.FiducialResult> tags) {
        List<HiveCell> out = new ArrayList<>();
        String[] names = Constants.Vision.CELL_NAMES;
        int[][] idTable = Constants.Vision.CELL_TAG_IDS;

        for (int c = 0; c < names.length; c++) {
            int seen = 0, withPose = 0;
            double sumTx = 0, sumTy = 0, sumDist = 0, sumRoll = 0;

            for (LLResultTypes.FiducialResult t : tags) {
                if (!contains(idTable[c], t.getFiducialId())) continue;
                seen++;
                sumTx += t.getTargetXDegrees();
                sumTy += t.getTargetYDegrees();

                // 3D pose needs the tag size (3.25 in = 82.55 mm) set in the Limelight pipeline.
                Pose3D pose = t.getTargetPoseCameraSpace();
                if (pose != null) {
                    Position p = pose.getPosition().toUnit(DistanceUnit.INCH);
                    // Limelight camera space: x right, y down, z forward -> floor distance
                    double dist = Math.hypot(p.x, p.z);
                    if (dist > 0.1) {
                        sumDist += dist;
                        sumRoll += pose.getOrientation().getRoll(AngleUnit.DEGREES);
                        withPose++;
                    }
                }
            }
            if (seen == 0) continue;

            double dist = withPose > 0 ? sumDist / withPose : Double.NaN;
            double roll = withPose > 0
                    ? wrapDegrees(sumRoll / withPose - Constants.Vision.ROLL_OFFSET_DEG)
                    : 0; // no pose: assume upright so aiming still works; telemetry shows dist "-"
            out.add(new HiveCell(names[c], seen, sumTx / seen, sumTy / seen, dist, roll));
        }
        return out;
    }

    // ---------------------------------------------------------------- queries

    public boolean isConnected() { return limelight != null; }

    /** Every HIVE CELL the camera can see right now (any alliance, any orientation). */
    public List<HiveCell> getVisibleCells() { return cells; }

    /** Raw tag IDs seen (for debugging the ID table). */
    public List<Integer> getRawTagIds() { return rawTagIds; }

    /**
     * Step 4 + 5: the CELL we should shoot at — our alliance, upright.
     * If both of ours look upright (bad roll calibration), pick the one with more
     * tags visible, then the closer one. Null if none.
     */
    public HiveCell getTargetCell(Alliance alliance) {
        HiveCell best = null;
        for (HiveCell c : cells) {
            if (!c.belongsTo(alliance) || !c.isUpright()) continue;
            if (best == null
                    || c.tagsSeen > best.tagsSeen
                    || (c.tagsSeen == best.tagsSeen && closer(c, best))) {
                best = c;
            }
        }
        return best;
    }

    // ---------------------------------------------------------------- helpers

    private static boolean closer(HiveCell a, HiveCell b) {
        if (Double.isNaN(a.distanceIn)) return false;
        return Double.isNaN(b.distanceIn) || a.distanceIn < b.distanceIn;
    }

    private static boolean contains(int[] ids, int id) {
        for (int i : ids) if (i == id) return true;
        return false;
    }

    private static double wrapDegrees(double deg) {
        while (deg > 180) deg -= 360;
        while (deg <= -180) deg += 360;
        return deg;
    }
}
