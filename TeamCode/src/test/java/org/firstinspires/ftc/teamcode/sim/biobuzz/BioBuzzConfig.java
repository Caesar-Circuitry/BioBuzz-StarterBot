package org.firstinspires.ftc.teamcode.sim.biobuzz;

/**
 * Every tunable number for the BioBuzz field simulation, in one place.
 *
 * Units: inches, seconds, radians (degrees where the name says Deg).
 * Coordinates: FTCodeSim/Pedro field coordinates. (0,0) is one field corner,
 * x and y run 0..fieldSize, z is height above the tiles.
 *
 * Where the numbers come from:
 *   MANUAL  the BIOBUZZ Competition Manual, Sections 9-10
 *   MODEL   measured from AdvantageScope's official 2026-2027 field model
 *           (model frame + 70.75 in = these coordinates; see BioBuzzField.validate())
 *   TUNE    physics guesses: adjust until the sim matches the real robot/field
 *
 * Field orientation (from the model): y = 0 is the audience wall, y = fieldSize
 * the rear wall, the red alliance area is past x = 0, blue past x = fieldSize.
 */
public class BioBuzzConfig {

    // ======================= FIELD =======================

    /** MODEL: inside of the walls (tiles are 141.2 in; FTCodeSim's BIOBUZZ boundary uses 141.3). */
    public double fieldSize = 141.3;
    public double gravity = 386.09;               // in/s^2

    // ======================= BALLS =======================

    public double pollenDiameter = 2.8;           // MANUAL
    public double nectarDiameter = 3.6;           // MANUAL
    /** Weight of one nectar relative to one pollen (same foam, so ~ (3.6/2.8)^3). TUNE */
    public double nectarWeight = 2.125;

    public double floorRestitution = 0.35;        // TUNE: how bouncy on foam tiles
    public double wallRestitution = 0.45;         // TUNE
    public double ballRestitution = 0.5;          // TUNE: ball-on-ball
    public double robotRestitution = 0.25;        // TUNE: ball bouncing off the robot frame
    public double rollingDecel = 20;              // TUNE: in/s^2 slowdown while rolling on tiles
    public double airDrag = 0.05;                 // TUNE: fraction of speed lost per second in the air

    // ======================= HIVE =======================
    // One frame in the middle of the field holding a red and a blue hive.
    // Each hive is a see-saw: two cells, one facing up. Launch enough into the
    // up cell and it tips, the cells swap, and the old cell dumps its contents.

    public double hiveStructureX = 70.75;         // MODEL: centre of the field
    public double hiveStructureY = 70.75;
    /** MODEL: red hive centre is 12.75 in toward red, blue 12.75 in toward blue. */
    public double hiveSpacing = 25.5;
    /** MODEL: the hives sit side by side along x and tip along y (toward audience / rear). */
    public boolean hivesAlongX = true;
    /** MODEL: the red hive is on the low-x (red alliance) side. */
    public boolean redHiveOnLowSide = true;

    /** MODEL: pivot offset along the tip direction (the bearings sit exactly on the centre line). */
    public double hivePivotShift = 0;

    /** MANUAL + MODEL: height of the pivot axis (centre of the hive bearings). */
    public double pivotHeight = 43.95;
    /**
     * MODEL: cell box, outside dimensions, in the hive's own frame.
     * Width runs along the pivot axis (the manual's ~20 in opening), depth along the beam.
     */
    public double cellWidth = 21.06;
    public double cellDepth = 16.85;
    public double cellHeight = 15.79;
    /** MODEL: cell centre distance from the pivot along the beam, and height above it. */
    public double cellCenterAlong = 13.13;
    public double cellCenterAbovePivot = 5.18;
    /** MODEL: wall and floor thickness, so balls inside sit where the model's nectar sits. */
    public double cellWallThickness = 0.7;
    public double cellFloorThickness = 1.35;
    /**
     * MODEL: the inside of a cell (where a ball counts as in): between the back wall and the
     * outer rib (this far from the pivot along the beam) and up to the rim of the top opening.
     * The opening is the cell's top face: ~20 in wide x 12 in deep, matching the manual.
     */
    public double cellInsideAlongMin = 9.4;
    public double cellInsideAlongMax = 21.4;
    public double cellInsideWidth = 20.0;
    public double cellRimAbovePivot = 12.6;
    /** MODEL: the starting nectar row sits this far toward the pivot from the cell centre. */
    public double cellFirstRowTowardPivot = 1.89;
    /** MANUAL: the frame's footprint at its base (used to keep the robot and spawns clear). */
    public double hiveFrameWidth = 49.46;
    public double hiveFrameDepth = 38.95;

    // ---- hive motion ----
    /** MODEL: the hive rests 30 degrees from level; the raised cell is the one that catches. */
    public double hiveRestAngleDeg = 30;
    /** TUNE: rad/s^2 pushing the hive toward the other side once it lets go. */
    public double hiveSnapAccel = 8;
    /** TUNE: extra rad/s^2 per pollen-equivalent still in the falling cell. */
    public double hiveWeightAccel = 0.8;
    /** TUNE: fraction of angular speed lost per second while swinging. */
    public double hiveDamping = 0.5;
    /** TUNE: bounciness of the damper at the end stop (0 = dead stop). */
    public double hiveDamperRestitution = 0.25;
    /** TUNE: the falling cell dumps its balls this many degrees past level. */
    public double hiveDumpAngleDeg = 5;

    /** TUNE: pollen-equivalents in the up cell that make a hive tip. Manual gives no number. */
    public double tipThreshold = 10.0;

    /**
     * MODEL: which cell is raised at the start (-1 = audience side, +1 = rear side).
     * Red starts with its audience cell up, blue with its rear ("scoring") cell up.
     */
    public int redInitialUpSide = -1;
    public int blueInitialUpSide = 1;

    public int nectarPerCellStart = 3;            // MANUAL
    public int nectarStagedPerAlliance = 5;       // MANUAL: one enters per tip

    // ======================= FLOWERS =======================

    /** MODEL: {x, y} of the 4 flowers, one per wall in a pinwheel. */
    public double[][] flowerPositions = {
            {2.71, 47.36},      // red wall
            {47.36, 138.79},    // rear wall
            {94.14, 2.71},      // audience wall
            {138.79, 94.14},    // blue wall
    };
    public double flowerTopHeight = 21.5;         // MANUAL
    public double flowerOpeningDiameter = 4.0;    // MANUAL
    /** MODEL: outer radius of the four-pipe tube, used to bounce balls off it. */
    public double flowerBodyRadius = 3.1;
    /** MODEL: the bottom pollen sits on the tiles (the retrieval opening is the bottom 4.25 in). */
    public double flowerStackBaseHeight = 0;
    public int pollenPerFlowerStart = 4;          // MANUAL

    // ======================= GARDENS & ALLIANCE AREAS =======================

    public int pollenPerGardenStart = 4;          // MANUAL
    /**
     * MODEL: {xMin, yMin, xMax, yMax} of each garden, from the taped strips. Red is in the
     * audience/red corner, blue in the rear/blue corner. 1 point per piece resting in it.
     */
    public double[] redGarden = {0, 0, 23.35, 3.0};
    public double[] blueGarden = {117.95, 138.3, 141.3, 141.3};
    /** MODEL: centre of each taped loading zone, where a released nectar is put on the field. */
    public double[] redNectarSpawn = {5.65, 106.0};
    public double[] blueNectarSpawn = {135.85, 35.5};

    // ======================= ROBOT =======================

    // Robot box the balls bounce off. StarterBotGeometry.applyTo(this) sets these from the
    // StarterBot model (robot frame: x forward from the drivetrain centre, y left, z up).
    public double robotLength = 17.8;             // front-to-back
    public double robotWidth = 17.8;
    public double robotHeight = 12.69;            // top of the hood; balls above this fly over
    /** The frame's centre is this far in front of the drivetrain centre (front bumper at +9.77). */
    public double robotCenterOffsetX = 0.87;
    /**
     * Where the robot is put at the start of every OpMode. Facing +y = PI/2.
     * Default: a spot the StarterBot can score the red hive from (found by sweeping the
     * field with the measured StarterBot geometry), back of the robot toward the red hive.
     * From here, spin the flywheel to 84% (about 2350 ticks/s), then feed one ball every
     * ~0.6 s: all 4 preloads go in and the hive tips. (42..46, 48..52) along the same line
     * works too; the mirror-image blue spot is about (99.5, 93.5) facing 44 degrees.
     */
    public double startX = 42;
    public double startY = 48;
    public double startHeading = Math.toRadians(224);

    public int preloadPollen = 4;                 // MANUAL

    // Intake and launcher settings live in SimIntakeConfig / SimLauncherConfig,
    // set up in SimulateMecanumBase next to the drivetrain.

    // ======================= SIM =======================

    /**
     * Compare the starting layout to positions measured from AdvantageScope's
     * official 2026-2027 field model (see FieldReference). Turn off if you
     * deliberately change the layout.
     */
    public boolean compareToFieldModel = true;

    /** Largest physics step. Smaller = more accurate, slower. */
    public double maxSubstep = 0.002;
    public long randomSeed = 1234;
}
