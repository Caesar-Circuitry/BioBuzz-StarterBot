package org.firstinspires.ftc.teamcode.sim.biobuzz;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.node.ArrayNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import org.codeblooded.ftcodesim.ascope.AdvantageScopeRunner;
import org.codeblooded.ftcodesim.ascope.SeasonField;
import org.psilynx.psikit.core.Logger;
import org.psilynx.psikit.core.wpi.math.Pose3d;
import org.psilynx.psikit.core.wpi.math.Rotation3d;
import org.psilynx.psikit.core.wpi.math.Translation3d;

import java.util.ArrayList;
import java.util.List;

/**
 * Draws the BioBuzz field in AdvantageScope. This is the ONLY BioBuzz class that
 * uses FTCodeSim's logger (PsiKit). Delete it and the update() call in
 * BioBuzzFieldSim and the physics still runs; you just won't see it.
 *
 * What gets logged (all under RealOutputs/BioBuzz/...):
 *   Pieces/Pollen, Pieces/RedNectar, Pieces/BlueNectar   one Translation3d[] list each
 *   Hive/RED/pose, Hive/BLUE/pose                        Pose3d of each hive (pivot + tilt)
 *   Hive/RED/outline, Hive/BLUE/outline                  rotating wireframe of each hive
 *   Hive/.../angleDeg, phase, tips, upCellWeight         for line graphs
 *   Score/..., Robot/...
 *
 * Two AdvantageScope 3D tabs are set up:
 *   "3D BioBuzz"     the field copy without hives + the real hive models, rotating
 *                    (needs BioBuzzAssets installed and AdvantageScope restarted once)
 *   "3D FTCodeSim"   FTCodeSim's own tab on the stock field, with the wireframes
 */
public class BioBuzzVisualizer {
    /** Game-piece model names, confirmed from AdvantageScope's 2026-2027 field config.json. */
    public String pollenVariant = "Pollen";
    public String redNectarVariant = "Nectar (Red)";
    public String blueNectarVariant = "Nectar (Blue)";
    public SeasonField seasonField = SeasonField.BIOBUZZ;
    /**
     * Robot model for FTCodeSim's robot in the "3D BioBuzz" tab. null = use the model in
     * AScope_Assets/Robot_StarterBot (its config.json "name"), or FTCodeSim's default if none.
     */
    public String robotModel = null;

    /** How often to log, in seconds. Physics runs ~1000x a second; drawing 50x is plenty. */
    public double logPeriod = 0.02;

    private double sinceLastLog = Double.MAX_VALUE;
    private boolean sourcesAdded = false;

    private static final String PIECES = "BioBuzz/Pieces/";
    private static final String HIVE = "BioBuzz/Hive/";
    private static final double INCHES_PER_METER = 39.37;
    private static final double HALF_FIELD = 141.5 / 2;   // same constants FTCodeSim uses

    public void update(BioBuzzField field, double deltaTime) {
        if (!sourcesAdded && AdvantageScopeRunner.INSTANCE != null) {
            addAdvantageScopeSources();
            sourcesAdded = true;
        }

        sinceLastLog += deltaTime;
        if (sinceLastLog < logPeriod) return;
        sinceLastLog = 0;

        Logger.recordOutput(PIECES + "Pollen", positions(field.pollen));
        Logger.recordOutput(PIECES + "RedNectar", positions(field.redNectar));
        Logger.recordOutput(PIECES + "BlueNectar", positions(field.blueNectar));

        for (Hive hive : field.hives.values()) {
            String k = HIVE + hive.alliance + "/";
            Logger.recordOutput(k + "pose", pose(hive));
            Logger.recordOutput(k + "outline", outline(hive));
            Logger.recordOutput(k + "angleDeg", Math.toDegrees(hive.angle));
            Logger.recordOutput(k + "phase", hive.phase.name());
            Logger.recordOutput(k + "upSide", hive.upSide());
            Logger.recordOutput(k + "tips", hive.tips);
            Logger.recordOutput(k + "upCellWeight", hive.upCellWeight());
        }

        for (Alliance a : Alliance.values()) {
            Score s = field.score(a);
            String k = "BioBuzz/Score/" + a + "/";
            Logger.recordOutput(k + "total", s.total());
            Logger.recordOutput(k + "tips", s.tips);
            Logger.recordOutput(k + "piecesInCells", s.piecesInCells);
            Logger.recordOutput(k + "flowersOwned", s.flowersOwned);
            Logger.recordOutput(k + "piecesInGarden", s.piecesInGarden);
            Logger.recordOutput(k + "stagedNectar", field.stagedNectar(a));
        }

        Logger.recordOutput("BioBuzz/Robot/ballsHeld", field.robotStorage.size());
    }

    /** Every piece of one type that is on the field (staged nectar is left out until released). */
    private static Translation3d[] positions(List<Ball> balls) {
        List<Translation3d> out = new ArrayList<>(balls.size());
        for (Ball b : balls) {
            if (b.state == Ball.State.STAGED) continue;
            out.add(toFtcMeters(b.x, b.y, b.z));
        }
        return out.toArray(new Translation3d[0]);
    }

    /**
     * One continuous line that traces both cell boxes and the beam between them,
     * so AdvantageScope can draw the hive as a single "trajectory" that rotates.
     */
    private static Translation3d[] outline(Hive hive) {
        List<Translation3d> pts = new ArrayList<>();
        // visits all 12 edges of a box: bottom loop, up, top loop, then the 3 remaining verticals
        int[] path = {0, 1, 2, 3, 0, 4, 5, 1, 5, 6, 2, 6, 7, 3, 7, 4};
        double[][] a = hive.cellCorners(-1);
        double[][] b = hive.cellCorners(1);
        for (int i : path) pts.add(toFtcMeters(a[i]));
        double[] pivotA = hive.toWorld(hive.cellCenterAlong(-1), 0, 0);
        double[] pivot = hive.toWorld(0, 0, 0);
        double[] pivotB = hive.toWorld(hive.cellCenterAlong(1), 0, 0);
        pts.add(toFtcMeters(pivotA));
        pts.add(toFtcMeters(pivot));
        pts.add(toFtcMeters(pivotB));
        for (int i : path) pts.add(toFtcMeters(b[i]));
        return pts.toArray(new Translation3d[0]);
    }

    /**
     * The hive models' origin is on the pivot axis with the hive level, so the pose is
     * the pivot position plus a pitch (rotation about the field's Y axis in
     * AdvantageScope's frame, which is the pivot axis) equal to the hive angle.
     */
    private static Pose3d pose(Hive hive) {
        Translation3d p = toFtcMeters(hive.x, hive.y, hive.pivotHeight());
        return new Pose3d(p.getX(), p.getY(), p.getZ(), new Rotation3d(0, hive.angle, 0));
    }

    /** Pedro inches -> FTC field metres (centre origin), the frame AdvantageScope's FTC fields use. */
    private static Translation3d toFtcMeters(double x, double y, double z) {
        return new Translation3d(
                -(y - HALF_FIELD) / INCHES_PER_METER,
                (x - HALF_FIELD) / INCHES_PER_METER,
                z / INCHES_PER_METER);
    }

    private static Translation3d toFtcMeters(double[] p) {
        return toFtcMeters(p[0], p[1], p[2]);
    }

    private void addAdvantageScopeSources() {
        AdvantageScopeRunner ascope = AdvantageScopeRunner.INSTANCE;
        ArrayNode sources = ascope.fieldView3D(seasonField);
        ObjectMapper mapper = new ObjectMapper();

        // remove sources from older versions of this sim (one per ball)
        for (int i = sources.size() - 1; i >= 0; i--) {
            JsonNode k = sources.get(i).get("logKey");
            if (k == null) continue;
            String key = k.asText();
            if (key.startsWith("RealOutputs/" + PIECES) && !key.equals("RealOutputs/" + PIECES + "Pollen")
                    && !key.equals("RealOutputs/" + PIECES + "RedNectar") && !key.equals("RealOutputs/" + PIECES + "BlueNectar")) {
                sources.remove(i);
            }
        }

        addSource(sources, mapper, PIECES + "Pollen", "gamePiece", "Translation3d[]",
                mapper.createObjectNode().put("variant", pollenVariant));
        addSource(sources, mapper, PIECES + "RedNectar", "gamePiece", "Translation3d[]",
                mapper.createObjectNode().put("variant", redNectarVariant));
        addSource(sources, mapper, PIECES + "BlueNectar", "gamePiece", "Translation3d[]",
                mapper.createObjectNode().put("variant", blueNectarVariant));
        addSource(sources, mapper, HIVE + "RED/outline", "trajectory", "Translation3d[]",
                mapper.createObjectNode().put("color", "#ff0000").put("size", "bold"));
        addSource(sources, mapper, HIVE + "BLUE/outline", "trajectory", "Translation3d[]",
                mapper.createObjectNode().put("color", "#0000ff").put("size", "bold"));

        addBioBuzzTab(ascope, sources, mapper);
        ascope.saveConfig();
    }

    /** A tab on the hive-less field showing the real hive models, plus FTCodeSim's robot. */
    private void addBioBuzzTab(AdvantageScopeRunner ascope, ArrayNode ftcodesimSources, ObjectMapper mapper) {
        ObjectNode tabState = (ObjectNode) ascope.get("hubs/0/state/tabs");
        ArrayNode tabs = (ArrayNode) tabState.get("tabs");
        ObjectNode tab = null;
        int index = -1;
        for (int i = 0; i < tabs.size(); i++) {
            JsonNode title = tabs.get(i).get("title");
            if (title != null && TAB_TITLE.equals(title.asText())) { tab = (ObjectNode) tabs.get(i); index = i; break; }
        }
        if (tab == null) {
            tab = mapper.createObjectNode();
            tab.put("type", 3)
                    .put("title", TAB_TITLE)
                    .put("controllerUUID", "biobuzzsimtab0000000000000000001")
                    .put("controlsHeight", 100);
            tab.set("controller", mapper.createObjectNode().put("game", BioBuzzAssets.FIELD_ID).set("sources", mapper.createArrayNode()));
            index = Math.min(1, tabs.size());
            tabs.insert(index, tab);
        }
        ObjectNode controller = (ObjectNode) tab.get("controller");
        controller.put("game", BioBuzzAssets.FIELD_ID);
        ArrayNode sources = (ArrayNode) controller.get("sources");

        // FTCodeSim's robot (and anything else it logs), but not the wireframes
        for (JsonNode s : ftcodesimSources) {
            JsonNode k = s.get("logKey");
            if (k == null || k.asText().startsWith("RealOutputs/" + HIVE)) continue;
            String key = k.asText().substring(k.asText().startsWith("RealOutputs/") ? "RealOutputs/".length() : 0);
            JsonNode type = s.get("type"), logType = s.get("logType"), options = s.get("options");
            if (type == null || logType == null) continue;
            ObjectNode opts = options instanceof ObjectNode ? ((ObjectNode) options).deepCopy() : mapper.createObjectNode();
            String model = robotModel != null ? robotModel : BioBuzzAssets.starterBotModel;
            if ("robot".equals(type.asText()) && model != null) opts.put("model", model);
            addSource(sources, mapper, key, type.asText(), logType.asText(), opts);
        }
        String model = robotModel != null ? robotModel : BioBuzzAssets.starterBotModel;
        if (model != null) {
            for (JsonNode s : sources) {
                JsonNode t = s.get("type"), k = s.get("logKey");
                if (t != null && "robot".equals(t.asText()) && k != null && !k.asText().startsWith("RealOutputs/" + HIVE)
                        && s.get("options") instanceof ObjectNode) {
                    ((ObjectNode) s.get("options")).put("model", model);
                }
            }
        }
        addSource(sources, mapper, HIVE + "RED/pose", "robot", "Pose3d",
                mapper.createObjectNode().put("model", BioBuzzAssets.RED_HIVE_MODEL));
        addSource(sources, mapper, HIVE + "BLUE/pose", "robot", "Pose3d",
                mapper.createObjectNode().put("model", BioBuzzAssets.BLUE_HIVE_MODEL));

        tabState.put("selected", index);
    }

    private static final String TAB_TITLE = "3D BioBuzz";

    private static void addSource(ArrayNode sources, ObjectMapper mapper, String key, String type,
                                  String logType, ObjectNode options) {
        String logKey = "RealOutputs/" + key;
        for (JsonNode s : sources) {
            JsonNode k = s.get("logKey");
            if (k != null && k.asText().equals(logKey)) return;
        }
        ObjectNode source = mapper.createObjectNode();
        source.put("type", type)
                .put("logKey", logKey)
                .put("logType", logType)
                .put("visible", true);
        source.set("options", options);
        sources.add(source);
    }
}
