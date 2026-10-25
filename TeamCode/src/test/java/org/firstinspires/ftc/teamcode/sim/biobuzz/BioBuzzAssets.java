package org.firstinspires.ftc.teamcode.sim.biobuzz;

import java.io.File;
import java.io.IOException;
import java.net.URL;
import java.nio.file.Files;
import java.nio.file.StandardCopyOption;
import java.util.ArrayList;
import java.util.List;

/**
 * Installs AdvantageScope assets (the same way FTCodeSim installs its robot models):
 * copies every TYPE_NAME folder from
 *   TeamCode/src/test/resources/advantagescope   (the BioBuzz field and hives)
 *   AScope_Assets                               (your own assets, e.g. Robot_StarterBot)
 * into AdvantageScope's user-assets folder.
 *
 *   Field3d_BioBuzzSim      the official 2026-2027 field with both hives removed
 *   Robot_BioBuzzRedHive    the red hive, origin on its pivot axis, drawn level
 *   Robot_BioBuzzBlueHive   the blue hive, same
 *
 * The models were cut out of AdvantageScope's own 2026-2027 field model, so the
 * sim can rotate the hives (see BioBuzzVisualizer). Only changed files are copied.
 * AdvantageScope reads user assets when it starts, so restart it once after the
 * first install.
 */
public final class BioBuzzAssets {
    private BioBuzzAssets() {}

    public static final String FIELD_NAME = "2026-2027 Field (BioBuzz Sim)";
    public static final String FIELD_ID = "FTC:" + FIELD_NAME;
    public static final String RED_HIVE_MODEL = "BioBuzz Red Hive";
    public static final String BLUE_HIVE_MODEL = "BioBuzz Blue Hive";

    /** Name of the robot model in AScope_Assets/Robot_StarterBot (null if not found). Set by install(). */
    public static volatile String starterBotModel = null;

    /** Copy any new or changed asset files. Returns true if something was copied. Never throws. */
    public static boolean install() {
        try {
            List<File> sources = new ArrayList<>();
            File simAssets = findSourceFolder();
            if (simAssets != null) sources.add(simAssets);
            else System.out.println("[BioBuzz] BioBuzz AdvantageScope assets not found (expected TeamCode/src/test/resources/advantagescope); hives will show as wireframes only");
            File projectAssets = findProjectAssetsFolder();
            if (projectAssets != null) sources.add(projectAssets);
            if (sources.isEmpty()) return false;

            File target = new File(advantageScopeFolder(), "userAssets");
            List<String> copied = new ArrayList<>();
            for (File source : sources) {
                File[] assets = source.listFiles(f -> f.isDirectory() && f.getName().contains("_"));
                if (assets == null) continue;
                for (File asset : assets) {
                    File[] files = asset.listFiles(File::isFile);
                    if (files == null) continue;
                    if (asset.getName().startsWith("Robot_") && asset.getName().toLowerCase().contains("starterbot")) {
                        starterBotModel = readName(new File(asset, "config.json"));
                    }
                    File dest = new File(target, asset.getName());
                    for (File f : files) {
                        File d = new File(dest, f.getName());
                        if (d.exists() && d.length() == f.length() && d.lastModified() >= f.lastModified()) continue;
                        Files.createDirectories(dest.toPath());
                        Files.copy(f.toPath(), d.toPath(), StandardCopyOption.REPLACE_EXISTING, StandardCopyOption.COPY_ATTRIBUTES);
                        copied.add(asset.getName() + "/" + f.getName());
                    }
                }
            }
            if (starterBotModel != null) System.out.println("[BioBuzz] Robot model for the 3D BioBuzz tab: " + starterBotModel);
            if (copied.isEmpty()) {
                System.out.println("[BioBuzz] AdvantageScope assets up to date in " + target);
                return false;
            }
            System.out.println("[BioBuzz] Installed " + copied.size() + " AdvantageScope asset file(s) into " + target + ": " + copied);
            System.out.println("[BioBuzz] >>> Restart AdvantageScope once so it loads the new field, hive and robot models <<<");
            return true;
        } catch (IOException | RuntimeException e) {
            System.out.println("[BioBuzz] Could not install AdvantageScope assets: " + e);
            return false;
        }
    }

    /** The "name" from an AdvantageScope config.json, without needing a JSON library. */
    static String readName(File config) {
        try {
            String json = new String(Files.readAllBytes(config.toPath()), java.nio.charset.StandardCharsets.UTF_8);
            java.util.regex.Matcher m = java.util.regex.Pattern.compile("\"name\"\\s*:\\s*\"([^\"]+)\"").matcher(json);
            return m.find() ? m.group(1) : null;
        } catch (IOException e) {
            return null;
        }
    }

    /** The project's own AScope_Assets folder (next to TeamCode). */
    static File findProjectAssetsFolder() {
        String dir = System.getProperty("user.dir", ".");
        for (String c : new String[]{"AScope_Assets", "../AScope_Assets"}) {
            File f = new File(dir, c);
            if (f.isDirectory()) return f;
        }
        return null;
    }

    /** The project's asset folder, from the working directory or the test classpath. */
    static File findSourceFolder() {
        String dir = System.getProperty("user.dir", ".");
        String[] candidates = {
                "src/test/resources/advantagescope",           // working dir = TeamCode (Android Studio default)
                "TeamCode/src/test/resources/advantagescope",  // working dir = project root
        };
        for (String c : candidates) {
            File f = new File(dir, c);
            if (f.isDirectory()) return f;
        }
        try {
            URL url = BioBuzzAssets.class.getClassLoader().getResource("advantagescope");
            if (url != null && "file".equals(url.getProtocol())) {
                File f = new File(url.toURI());
                if (f.isDirectory()) return f;
            }
        } catch (Exception ignored) {
        }
        return null;
    }

    /** Same location FTCodeSim uses. Override with -Dbiobuzz.ascopeFolder=... if needed. */
    static File advantageScopeFolder() {
        String override = System.getProperty("biobuzz.ascopeFolder");
        if (override != null) return new File(override);
        String os = System.getProperty("os.name", "").toLowerCase();
        String home = System.getProperty("user.home");
        if (os.contains("win")) {
            String appData = System.getenv("APPDATA");
            return appData != null ? new File(appData, "AdvantageScope")
                    : new File(new File(new File(home, "AppData"), "Roaming"), "AdvantageScope");
        } else if (os.contains("mac")) {
            return new File(new File(new File(home, "Library"), "Application Support"), "AdvantageScope");
        } else {
            String xdg = System.getenv("XDG_CONFIG_HOME");
            return xdg != null ? new File(xdg, "AdvantageScope") : new File(new File(home, ".config"), "AdvantageScope");
        }
    }
}
