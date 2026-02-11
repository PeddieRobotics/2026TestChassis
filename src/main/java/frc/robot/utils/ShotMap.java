package frc.robot.utils;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileReader;
import java.io.IOException;
import java.io.InputStream;
import java.util.StringTokenizer;

import edu.wpi.first.wpilibj.Filesystem;

public class ShotMap {
    public static record ShotMapValue(float exit_v, float pitch, float flightTime) {};
    
    private static final double MIN_DIST = 0.3;
    private static final double MAX_DIST = 7;
    private static final double DIST_INTERVAL = 0.1;

    private static final double MIN_VR = -3;
    private static final double MAX_VR = 3;
    private static final double VR_INTERVAL = 0.1;
    
    private static final int NUM_DIST = (int) ((MAX_DIST - MIN_DIST) / DIST_INTERVAL + 1);
    private static final int NUM_VR = (int) ((MAX_VR - MIN_VR) / VR_INTERVAL + 1);
    
    private static float shotMap[][][];

    // private static float lerp(float a, float b, float f) {
    //     return a * (1.0f - f) + (b * f);
    // }

    public static void initShotMap(String fileName) throws IOException {
        File deployDir = Filesystem.getDeployDirectory();
        File propFile = new File(deployDir, fileName);

		BufferedReader br = new BufferedReader(new FileReader(propFile));
		StringTokenizer st = new StringTokenizer(br.readLine());
        
        shotMap = new float[NUM_DIST][NUM_VR][3];
        
        for (int i = 0; i < NUM_DIST; i++) {
            for (int j = 0; j < NUM_VR; j++) {
                // exit_v then pitch
                shotMap[i][j][0] = Float.parseFloat(st.nextToken());
                shotMap[i][j][1] = Float.parseFloat(st.nextToken());
                
                // TODO: get new shot table with time of flight
                shotMap[i][j][2] = 0f;

                // shotMap[i][j][2] = Float.parseFloat(st.nextToken());
            }
        }

        br.close();
    }
    
    public static double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }

    public static ShotMapValue queryShotMap(double dist, double v_r) {
        dist = clamp(dist, MIN_DIST, MAX_DIST);
        v_r = clamp(v_r, MIN_VR, MAX_VR);

        double dist_index_frac = (dist - MIN_DIST) / DIST_INTERVAL;
        double vr_index_frac = (v_r - MIN_VR) / VR_INTERVAL;
        
        // TODO: consider if interpolation is needed here
        int dist_index = (int) Math.round(dist_index_frac);
        int vr_index = (int) Math.round(vr_index_frac);
        
        float[] mapEntry = shotMap[dist_index][vr_index];
        return new ShotMapValue(mapEntry[0], mapEntry[1], mapEntry[2]);
    }
}
