/*
 * Copyright 2014 Google Inc. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

package com.projecttango.examples.java.pointcloud;

import android.Manifest;
import android.content.Context;
import android.content.SharedPreferences;
import android.graphics.Color;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.hardware.display.DisplayManager;
import android.net.wifi.ScanResult;
import android.net.wifi.WifiManager;
import android.os.Bundle;
import android.support.v7.app.AppCompatActivity;
import android.util.Log;
import android.view.Display;
import android.view.MotionEvent;
import android.view.View;
import android.widget.Button;
import android.widget.CompoundButton;
import android.widget.Switch;
import android.widget.TextView;
import android.widget.Toast;

import com.android.volley.AuthFailureError;
import com.android.volley.DefaultRetryPolicy;
import com.android.volley.Request;
import com.android.volley.RequestQueue;
import com.android.volley.Response;
import com.android.volley.VolleyError;
import com.android.volley.toolbox.StringRequest;
import com.android.volley.toolbox.Volley;
import com.google.atap.tangoservice.Tango;
import com.google.atap.tangoservice.TangoConfig;
import com.google.atap.tangoservice.TangoCoordinateFramePair;
import com.google.atap.tangoservice.TangoErrorException;
import com.google.atap.tangoservice.TangoEvent;
import com.google.atap.tangoservice.TangoInvalidException;
import com.google.atap.tangoservice.TangoOutOfDateException;
import com.google.atap.tangoservice.TangoPointCloudData;
import com.google.atap.tangoservice.TangoPoseData;
import com.projecttango.tangosupport.TangoPointCloudManager;
import com.projecttango.tangosupport.TangoSupport;
import com.projecttango.tangosupport.ux.TangoUx;
import com.projecttango.tangosupport.ux.UxExceptionEvent;
import com.projecttango.tangosupport.ux.UxExceptionEventListener;
import com.vistrav.ask.Ask;

import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.RealMatrix;
import org.rajawali3d.scene.ASceneFrameCallback;
import org.rajawali3d.surface.RajawaliSurfaceView;
import org.rajawali3d.util.ArrayUtils;

import java.nio.FloatBuffer;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Calendar;
import java.util.Date;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Timer;
import java.util.TimerTask;

public class PointCloudActivity extends AppCompatActivity implements SensorEventListener {
    private static final String TAG = "Padmal";

    private static final String UX_EXCEPTION_EVENT_DETECTED = "Exception Detected: ";
    private static final String UX_EXCEPTION_EVENT_RESOLVED = "Exception Resolved: ";

    private static final int SECS_TO_MILLISECS = 1000;
    private static final double UPDATE_INTERVAL_MS = 200.0;
    private final String CLOUD_URL = "http://202.94.70.33/tango/insert_tango_point_cloud.php";
    private final String WIFI_URL = "http://202.94.70.33/tango/insert_tango_wifi_scan.php";
    private final String IMU_URL = "http://202.94.70.33/tango/insert_tango_raw_imu.php";
    private final String MODE = "Mode";
    private final long CLOUD_INTERVAL = 1000;
    private final long WIFI_INTERVAL = 5000;
    private final int MM = 10000;
    private Tango mTango;
    private TangoConfig mConfig;
    private TangoUx mTangoUx;
    private TangoPointCloudManager mPointCloudManager;
    private PointCloudRajawaliRenderer mRenderer;
    private RajawaliSurfaceView mSurfaceView;
    private TextView mPointCountTextView;
    // private TextView mAverageZTextView;
    private double mPointCloudPreviousTimeStamp;
    private boolean mIsConnected = false;
    private double mPointCloudTimeToNextUpdate = UPDATE_INTERVAL_MS;
    private int mDisplayRotation = 0;
    private SensorManager sensorManager;
    private Sensor GyroSensor, AccelSensor, MagenetSensor;
    private float[] OriReading;
    private float[] AccReading;
    private float[] GyroReading;
    private float[] MagReading;
    private float[] mGravity, mGeomagnetic;
    // Custom variables
    private TextView mTime, wifiStat, cloudStat, imuStat;
    private TextView mNodes, mTimeStamp;
    private Button runStop;
    private TextView Xv, Yv, Zv, Xtv, Ytv, Ztv, Ax, Ay, Az;
    private TextView oriX, oriY, oriZ, accX, accY, accZ, gyrX, gyrY, gyrZ, magX, magY, magZ;
    private Timer timer;
    private TangoPoseData lastPose;
    private TangoPointCloudData lastCloud;
    private double[] unityPose;
    private double[] completePose;
    private SharedPreferences logger;
    private WifiManager wifiManager;
    private List<ScanResult> scanResults;
    private int count = 0;

    private boolean started = false;

    private DefaultRetryPolicy retryPolicy;
    private RequestQueue queue;

    private int NodeCount = 0;
    private UxExceptionEventListener mUxExceptionListener = new UxExceptionEventListener() {
        @Override
        public void onUxExceptionEvent(UxExceptionEvent uxExceptionEvent) {
            String status = uxExceptionEvent.getStatus() == UxExceptionEvent.STATUS_DETECTED ?
                    UX_EXCEPTION_EVENT_DETECTED : UX_EXCEPTION_EVENT_RESOLVED;

            if (uxExceptionEvent.getType() == UxExceptionEvent.TYPE_LYING_ON_SURFACE) {
                Log.i(TAG, status + "Device lying on surface");
            }
            if (uxExceptionEvent.getType() == UxExceptionEvent.TYPE_FEW_DEPTH_POINTS) {
                Log.i(TAG, status + "Too few depth points");
            }
            if (uxExceptionEvent.getType() == UxExceptionEvent.TYPE_FEW_FEATURES) {
                Log.i(TAG, status + "Too few features");
            }
            if (uxExceptionEvent.getType() == UxExceptionEvent.TYPE_MOTION_TRACK_INVALID) {
                Log.i(TAG, status + "Invalid poses in MotionTracking");
            }
            if (uxExceptionEvent.getType() == UxExceptionEvent.TYPE_MOVING_TOO_FAST) {
                Log.i(TAG, status + "Moving too fast");
            }
            if (uxExceptionEvent.getType() == UxExceptionEvent.TYPE_FISHEYE_CAMERA_OVER_EXPOSED) {
                Log.i(TAG, status + "Fisheye Camera Over Exposed");
            }
            if (uxExceptionEvent.getType() == UxExceptionEvent.TYPE_FISHEYE_CAMERA_UNDER_EXPOSED) {
                Log.i(TAG, status + "Fisheye Camera Under Exposed");
            }
        }
    };

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_point_cloud);
        Ask.on(this).forPermissions(Manifest.permission.ACCESS_COARSE_LOCATION,
                Manifest.permission.ACCESS_FINE_LOCATION,
                Manifest.permission.ACCESS_WIFI_STATE,
                Manifest.permission.CHANGE_WIFI_STATE).go();

        AccReading = new float[3];
        MagReading = new float[3];
        OriReading = new float[3];
        GyroReading = new float[3];
        mGravity = new float[3];
        mGeomagnetic = new float[3];

        mPointCountTextView = (TextView) findViewById(R.id.point_count_textview);
        mTime = (TextView) findViewById(R.id.average_time);
        mNodes = (TextView) findViewById(R.id.nodes);
        Xv = (TextView) findViewById(R.id.x_vl);
        Yv = (TextView) findViewById(R.id.y_vl);
        Zv = (TextView) findViewById(R.id.z_vl);
        Xtv = (TextView) findViewById(R.id.xt_vl);
        Ytv = (TextView) findViewById(R.id.yt_vl);
        Ztv = (TextView) findViewById(R.id.zt_vl);
        Ax = (TextView) findViewById(R.id.ax);
        Ay = (TextView) findViewById(R.id.ay);
        Az = (TextView) findViewById(R.id.az);

        oriX = (TextView) findViewById(R.id.ori_x);
        oriY = (TextView) findViewById(R.id.ori_y);
        oriZ = (TextView) findViewById(R.id.ori_z);
        accX = (TextView) findViewById(R.id.acc_x);
        accY = (TextView) findViewById(R.id.acc_y);
        accZ = (TextView) findViewById(R.id.acc_z);
        gyrX = (TextView) findViewById(R.id.gyr_x);
        gyrY = (TextView) findViewById(R.id.gyr_y);
        gyrZ = (TextView) findViewById(R.id.gyr_z);
        magX = (TextView) findViewById(R.id.mag_x);
        magY = (TextView) findViewById(R.id.mag_y);
        magZ = (TextView) findViewById(R.id.mag_z);

        wifiStat = (TextView) findViewById(R.id.wifi_stat);
        cloudStat = (TextView) findViewById(R.id.cloud_stat);
        imuStat = (TextView) findViewById(R.id.imu_stat);
        unityPose = new double[6];
        completePose = new double[13];
        mTimeStamp = (TextView) findViewById(R.id.time_stamp);
        runStop = (Button) findViewById(R.id.run_stop);
        mSurfaceView = (RajawaliSurfaceView) findViewById(R.id.gl_surface_view);
        queue = Volley.newRequestQueue(this);

        logger = this.getSharedPreferences("Tango", Context.MODE_PRIVATE);

        mPointCloudManager = new TangoPointCloudManager();
        mTangoUx = setupTangoUxAndLayout();
        mRenderer = new PointCloudRajawaliRenderer(this);
        setupRenderer();

        wifiManager = (WifiManager) getSystemService(Context.WIFI_SERVICE);
        if (!wifiManager.isWifiEnabled()) {
            Toast.makeText(getApplicationContext(), "Enabling WIFI", Toast.LENGTH_SHORT).show();
            wifiManager.setWifiEnabled(true);
        }

        sensorManager = (SensorManager) getSystemService(Context.SENSOR_SERVICE);
        AccelSensor = sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
        MagenetSensor = sensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD);
        GyroSensor = sensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE);
        sensorManager.registerListener(this, AccelSensor, SensorManager.SENSOR_DELAY_UI);
        sensorManager.registerListener(this, MagenetSensor, SensorManager.SENSOR_DELAY_UI);
        sensorManager.registerListener(this, GyroSensor, SensorManager.SENSOR_DELAY_UI);

        DisplayManager displayManager = (DisplayManager) getSystemService(DISPLAY_SERVICE);
        if (displayManager != null) {
            displayManager.registerDisplayListener(new DisplayManager.DisplayListener() {
                @Override
                public void onDisplayAdded(int displayId) {
                }

                @Override
                public void onDisplayChanged(int displayId) {
                    synchronized (this) {
                        setDisplayRotation();
                    }
                }

                @Override
                public void onDisplayRemoved(int displayId) {
                }
            }, null);
        }
        retryPolicy = new DefaultRetryPolicy(10000, DefaultRetryPolicy.DEFAULT_MAX_RETRIES - 1, DefaultRetryPolicy.DEFAULT_BACKOFF_MULT);
        timer = new Timer();
        runStop.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                SimpleDateFormat sdf = new SimpleDateFormat("hh:mm:ss");
                String now = sdf.format(new Date()) + " " + String.valueOf(System.currentTimeMillis());
                started = !started;
                runStop.setText(started ? "Running!" : "Stopped!");
                mTimeStamp.setText(now);
            }
        });
    }

    @Override
    protected void onResume() {
        super.onResume();
        mRenderer.setFirstPersonView();
    }

    @Override
    protected void onStart() {
        super.onStart();
        mTangoUx.start();
        bindTangoService();
        timer = new Timer();
        timer.schedule(new TimerTask() {
            @Override
            public void run() {
                postAllData();
            }
        }, CLOUD_INTERVAL, CLOUD_INTERVAL);
    }

    private void postAllData() {
        if (started) {
            if (count == 4) {
                count = 0;
                postWiFiData();
            }
            count++;
            postCloudData();
            postIMUData();
        }
    }

    @Override
    protected void onStop() {
        super.onStop();
        timer.cancel();

        // Synchronize against disconnecting while the service is being used in the OpenGL
        // thread or in the UI thread.
        // NOTE: DO NOT lock against this same object in the Tango callback thread.
        // Tango.disconnect will block here until all Tango callback calls are finished.
        // If you lock against this object in a Tango callback thread it will cause a deadlock.
        synchronized (this) {
            try {
                mTangoUx.stop();
                mTango.disconnect();
                mIsConnected = false;
            } catch (TangoErrorException e) {
                Log.e(TAG, getString(R.string.exception_tango_error), e);
            }
        }
    }

    /**
     * Initialize Tango Service as a normal Android Service.
     */
    private void bindTangoService() {
        mTango = new Tango(PointCloudActivity.this, new Runnable() {
            @Override
            public void run() {
                synchronized (PointCloudActivity.this) {
                    try {
                        mConfig = setupTangoConfig(mTango);
                        mTango.connect(mConfig);
                        startupTango();
                        TangoSupport.initialize(mTango);
                        mIsConnected = true;
                        setDisplayRotation();
                    } catch (TangoOutOfDateException e) {
                        Log.e(TAG, getString(R.string.exception_out_of_date), e);
                    } catch (TangoErrorException e) {
                        Log.e(TAG, getString(R.string.exception_tango_error), e);
                        showsToastAndFinishOnUiThread(R.string.exception_tango_error);
                    } catch (TangoInvalidException e) {
                        Log.e(TAG, getString(R.string.exception_tango_invalid), e);
                        showsToastAndFinishOnUiThread(R.string.exception_tango_invalid);
                    }
                }
            }
        });
    }

    private TangoConfig setupTangoConfig(Tango tango) {
        TangoConfig config = tango.getConfig(TangoConfig.CONFIG_TYPE_DEFAULT);
        config.putBoolean(TangoConfig.KEY_BOOLEAN_DEPTH, true);
        config.putInt(TangoConfig.KEY_INT_DEPTH_MODE, TangoConfig.TANGO_DEPTH_MODE_POINT_CLOUD);
        return config;
    }

    private void startupTango() {
        ArrayList<TangoCoordinateFramePair> framePairs = new ArrayList<>();

        framePairs.add(new TangoCoordinateFramePair(TangoPoseData.COORDINATE_FRAME_START_OF_SERVICE,
                TangoPoseData.COORDINATE_FRAME_DEVICE));

        mTango.connectListener(framePairs, new Tango.TangoUpdateCallback() {
            @Override
            public void onPoseAvailable(TangoPoseData pose) {
                lastPose = pose;
                // Passing in the pose data to UX library produce exceptions.
                if (mTangoUx != null) {
                    mTangoUx.updatePoseStatus(pose.statusCode);
                }
            }


            @Override
            public void onPointCloudAvailable(TangoPointCloudData pointCloud) {
                lastCloud = pointCloud;
                if (mTangoUx != null) {
                    mTangoUx.updatePointCloud(pointCloud);
                }
                mPointCloudManager.updatePointCloud(pointCloud);
                final double currentTimeStamp = pointCloud.timestamp;
                final double pointCloudFrameDelta =
                        (currentTimeStamp - mPointCloudPreviousTimeStamp) * SECS_TO_MILLISECS;
                mPointCloudPreviousTimeStamp = currentTimeStamp;
                mPointCloudTimeToNextUpdate -= pointCloudFrameDelta;

                if (mPointCloudTimeToNextUpdate < 0.0) {
                    mPointCloudTimeToNextUpdate = UPDATE_INTERVAL_MS;
                    final String pointCountString = Integer.toString(pointCloud.numPoints);
                    runOnUiThread(new Runnable() {
                        @Override
                        public void run() {
                            mPointCountTextView.setText(pointCountString);
                            // mAverageZTextView.setText("Waiting");
                            // mAverageZTextView.setBackgroundColor(Color.TRANSPARENT);
                            wifiStat.setText("W W");
                            wifiStat.setBackgroundColor(Color.TRANSPARENT);
                            cloudStat.setText("C W");
                            cloudStat.setBackgroundColor(Color.TRANSPARENT);
                            imuStat.setText("I W");
                            imuStat.setBackgroundColor(Color.TRANSPARENT);
                            try {
                                double[] DATA = newMatrix(lastPose.getRotationAsFloats()[0],
                                        lastPose.getRotationAsFloats()[1],
                                        lastPose.getRotationAsFloats()[2],
                                        lastPose.getRotationAsFloats()[3]);
                                unityPose = DATA;
                                double[] oldPose = {(double) lastPose.getTranslationAsFloats()[0],
                                        (double) lastPose.getTranslationAsFloats()[1],
                                        (double) lastPose.getTranslationAsFloats()[2],
                                        (double) lastPose.getRotationAsFloats()[0],
                                        (double) lastPose.getRotationAsFloats()[1],
                                        (double) lastPose.getRotationAsFloats()[2],
                                        (double) lastPose.getRotationAsFloats()[3]};
                                completePose = ArrayUtils.concatAllDouble(oldPose, DATA);
                                String X = String.format("%.7f", DATA[0]);
                                Xv.setText(X);
                                Xtv.setText(String.format("%.7f", lastPose.getTranslationAsFloats()[0]));
                                Ax.setText(String.format("%.7f", DATA[3]));
                                String Y = String.format("%.7f", DATA[1]);
                                Yv.setText(Y);
                                Ytv.setText(String.format("%.7f", lastPose.getTranslationAsFloats()[1]));
                                Ay.setText(String.format("%.7f", DATA[4]));
                                String Z = String.format("%.7f", DATA[2]);
                                Zv.setText(Z);
                                Ztv.setText(String.format("%.7f", lastPose.getTranslationAsFloats()[2]));
                                Az.setText(String.format("%.7f", DATA[5]));
                            } catch (Exception e) {
                                Log.d("Padmal", "Error " + e.getMessage());
                            }
                        }
                    });
                }
            }

            @Override
            public void onTangoEvent(TangoEvent event) {
                if (mTangoUx != null) {
                    mTangoUx.updateTangoEvent(event);
                }
            }
        });
    }

    /**
     * Sets Rajawali surface view and its renderer. This is ideally called only once in onCreate.
     */
    public void setupRenderer() {
        mSurfaceView.setEGLContextClientVersion(2);
        mRenderer.getCurrentScene().registerFrameCallback(new ASceneFrameCallback() {
            @Override
            public void onPreFrame(long sceneTime, double deltaTime) {
                // NOTE: This will be executed on each cycle before rendering; called from the
                // OpenGL rendering thread.

                // Prevent concurrent access from a service disconnect through the onPause event.
                synchronized (PointCloudActivity.this) {
                    // Don't execute any Tango API actions if we're not connected to the service.
                    if (!mIsConnected) {
                        return;
                    }

                    // Update point cloud data.
                    TangoPointCloudData pointCloud = mPointCloudManager.getLatestPointCloud();

                    if (pointCloud != null) {
                        TangoSupport.TangoMatrixTransformData transform =
                                TangoSupport.getMatrixTransformAtTime(pointCloud.timestamp,
                                        TangoPoseData.COORDINATE_FRAME_START_OF_SERVICE,
                                        TangoPoseData.COORDINATE_FRAME_CAMERA_DEPTH,
                                        TangoSupport.TANGO_SUPPORT_ENGINE_OPENGL,
                                        TangoSupport.TANGO_SUPPORT_ENGINE_TANGO,
                                        TangoSupport.ROTATION_IGNORED);
                        if (transform.statusCode == TangoPoseData.POSE_VALID) {
                            mRenderer.updatePointCloud(pointCloud, transform.matrix);
                        }
                    }

                    // Update current camera pose.
                    try {
                        // Calculate the device pose. This transform is used to display
                        // frustum in third and top down view, and used to render camera pose in
                        // first person view.
                        TangoPoseData lastFramePose = TangoSupport.getPoseAtTime(0,
                                TangoPoseData.COORDINATE_FRAME_START_OF_SERVICE,
                                TangoPoseData.COORDINATE_FRAME_DEVICE,
                                TangoSupport.TANGO_SUPPORT_ENGINE_OPENGL,
                                TangoSupport.TANGO_SUPPORT_ENGINE_OPENGL,
                                mDisplayRotation);
                        if (lastFramePose.statusCode == TangoPoseData.POSE_VALID) {
                            mRenderer.updateCameraPose(lastFramePose);
                        }
                    } catch (TangoErrorException e) {
                        Log.e(TAG, "Could not get valid transform");
                    }
                }
            }

            @Override
            public boolean callPreFrame() {
                return true;
            }

            @Override
            public void onPreDraw(long sceneTime, double deltaTime) {

            }

            @Override
            public void onPostFrame(long sceneTime, double deltaTime) {

            }
        });
        mSurfaceView.setSurfaceRenderer(mRenderer);
    }

    private TangoUx setupTangoUxAndLayout() {
        TangoUx tangoUx = new TangoUx(this);
        tangoUx.setUxExceptionEventListener(mUxExceptionListener);
        return tangoUx;
    }

    /**
     * First Person button onClick callback.
     */
    public void onFirstPersonClicked(View v) {
        mRenderer.setFirstPersonView();
    }

    /**
     * Third Person button onClick callback.
     */
    public void onThirdPersonClicked(View v) {
        mRenderer.setThirdPersonView();
    }

    /**
     * Top-down button onClick callback.
     */
    public void onTopDownClicked(View v) {
        mRenderer.setTopDownView();
    }

    @Override
    public boolean onTouchEvent(MotionEvent event) {
        mRenderer.onTouchEvent(event);
        return true;
    }

    /**
     * Calculates the average depth from a point cloud buffer.
     *
     * @param pointCloudBuffer
     * @param numPoints
     * @return Average depth.
     */
    private float getAveragedDepth(FloatBuffer pointCloudBuffer, int numPoints) {
        float totalZ = 0;
        float averageZ = 0;
        if (numPoints != 0) {
            int numFloats = 4 * numPoints;
            for (int i = 2; i < numFloats; i = i + 4) {
                totalZ = totalZ + pointCloudBuffer.get(i);
            }
            averageZ = totalZ / numPoints;
        }
        return averageZ;
    }

    /**
     * Query the display's rotation.
     */
    private void setDisplayRotation() {
        Display display = getWindowManager().getDefaultDisplay();
        mDisplayRotation = display.getRotation();
    }

    /**
     * Display toast on UI thread.
     *
     * @param resId The resource id of the string resource to use. Can be formatted text.
     */
    private void showsToastAndFinishOnUiThread(final int resId) {
        runOnUiThread(new Runnable() {
            @Override
            public void run() {
                Toast.makeText(PointCloudActivity.this,
                        getString(resId), Toast.LENGTH_LONG).show();
                finish();
            }
        });
    }

    private void postCloudData() {
        try {
            StringRequest stringRequest = new StringRequest(Request.Method.POST, CLOUD_URL, new Response.Listener<String>() {
                @Override
                public void onResponse(String response) {
                    Calendar calendar = Calendar.getInstance();
                    mTime.setText(String.valueOf(calendar.get(Calendar.SECOND)));
                    String displayText = (response.contains("Data successfully created")) ? "C S" : "C F";
                    cloudStat.setText(displayText);
                    cloudStat.setBackgroundColor(displayText.contains("S") ? Color.GREEN : Color.RED);
                }
            }, new Response.ErrorListener() {
                @Override
                public void onErrorResponse(VolleyError error) {
                    /**/
                }
            }

            ) {
                @Override
                protected Map<String, String> getParams() throws AuthFailureError {
                    Map<String, String> body = new HashMap<>();
                    try {
                        // User ID *****************************************************************
                        body.put("user_id", "Padmal");
                        // Pose Data ***************************************************************
                        // orientation - As a quaternion of the pose of the target frame with
                        // reference to the base frame
                        // translation - ordered x, y, z of the pose of the target frame with
                        // reference to the base frame
                        String uniPose = Arrays.toString(completePose).replaceAll("\\s", "");
                        body.put("pose", uniPose.substring(1, uniPose.length() - 1));
                        // Tango Time **************************************************************
                        body.put("tango_time", String.valueOf(System.currentTimeMillis()));
                        // Point Cloud *************************************************************
                        StringBuilder cloudString = new StringBuilder();
                        cloudString.append(lastCloud.numPoints);
                        cloudString.append(",");
                        int remainingPoints = lastCloud.points.remaining();
                        TangoPointCloudData DATA = lastCloud;
                        for (int i = 0; i < remainingPoints; i++) {
                            if ((i + 1) % 4 != 0) {
                                double pnt = Math.floor(DATA.points.get(i) * 10000) / 10000;
                                cloudString.append(((int) (pnt * MM)));
                                cloudString.append(",");
                            }
                        }
                        cloudString.deleteCharAt(cloudString.length() - 1);
                        body.put("point_cloud", cloudString.toString());
                        // Posting *****************************************************************
                        lastPose = null;
                        return body;
                    } catch (Exception e) {
                        return null;
                    }
                }
            };
            stringRequest.setRetryPolicy(retryPolicy);
            queue.add(stringRequest);
        } catch (Exception e) {
            /**/
        }
    }

    private void postWiFiData() {
        try {
            StringRequest stringRequest = new StringRequest(Request.Method.POST, WIFI_URL, new Response.Listener<String>() {
                @Override
                public void onResponse(String response) {
                    Log.d("padmal", response);
                    Calendar calendar = Calendar.getInstance();
                    mTime.setText(String.valueOf(calendar.get(Calendar.SECOND)));
                    String displayText = (response.contains("Data successfully created")) ? "W S" : "W F";
                    wifiStat.setText(displayText);
                    wifiStat.setBackgroundColor(displayText.contains("W") ? Color.GREEN : Color.RED);
                    String nodeText = NodeCount < 2 ? NodeCount + " node" : NodeCount + " nodes";
                    mNodes.setText(nodeText);
                }
            }, new Response.ErrorListener() {
                @Override
                public void onErrorResponse(VolleyError error) {
                    /**/
                }
            }

            ) {
                @Override
                protected Map<String, String> getParams() throws AuthFailureError {
                    Map<String, String> body = new HashMap<>();
                    try {
                        wifiManager.startScan();
                        // User ID *****************************************************************
                        body.put("user_id", "Padmal");
                        // Pose Data ***************************************************************
                        String uniPose = Arrays.toString(completePose).replaceAll("\\s", "");
                        body.put("pose", uniPose.substring(1, uniPose.length() - 1));
                        // Tango Time **************************************************************
                        body.put("tango_time", String.valueOf(System.currentTimeMillis()));
                        // Point Cloud *************************************************************
                        scanResults = wifiManager.getScanResults();
                        int APs = scanResults.size();
                        NodeCount = APs;
                        StringBuilder wifiString = new StringBuilder();
                        wifiString.append(APs);
                        wifiString.append(",");
                        for (ScanResult result : scanResults) {
                            wifiString.append(result.SSID);
                            wifiString.append(",");
                            wifiString.append(result.frequency);
                            wifiString.append(",");
                            wifiString.append(result.level);
                            wifiString.append(",");
                            wifiString.append(result.BSSID);
                            wifiString.append(",");
                        }
                        wifiString.deleteCharAt(wifiString.length() - 1);
                        body.put("wifi_scan", wifiString.toString());
                        // Posting *****************************************************************
                        Log.d("Padmal", body.toString());
                        lastPose = null;
                        return body;
                    } catch (Exception e) {
                        return null;
                    }
                }
            };
            stringRequest.setRetryPolicy(retryPolicy);
            queue.add(stringRequest);
        } catch (Exception e) {
            /**/
        }
    }

    private void postIMUData() {
        try {
            StringRequest stringRequest = new StringRequest(Request.Method.POST, IMU_URL, new Response.Listener<String>() {
                @Override
                public void onResponse(String response) {
                    Log.d("padmal", response);
                    String displayText = (response.contains("Data successfully created")) ? "I S" : "I F";
                    imuStat.setText(displayText);
                    imuStat.setBackgroundColor(displayText.contains("S") ? Color.GREEN : Color.RED);
                }
            }, new Response.ErrorListener() {
                @Override
                public void onErrorResponse(VolleyError error) {
                    /**/
                }
            }

            ) {
                @Override
                protected Map<String, String> getParams() throws AuthFailureError {
                    Map<String, String> body = new HashMap<>();
                    try {
                        // User ID *****************************************************************
                        body.put("user_id", "Padmal");
                        // Pose Data ***************************************************************
                        String uniPose = Arrays.toString(completePose).replaceAll("\\s", "");
                        body.put("pose", uniPose.substring(1, uniPose.length() - 1));
                        // Tango Time **************************************************************
                        body.put("tango_time", String.valueOf(System.currentTimeMillis()));
                        // Orientation *************************************************************
                        // Acceleration ************************************************************
                        // Gyroscope ***************************************************************
                        // Magnetic ****************************************************************
                        String Orientation = Arrays.toString(OriReading).replaceAll("\\s", "");
                        body.put("orientation", Orientation.substring(1, Orientation.length() - 1));
                        String Acceleration = Arrays.toString(AccReading).replaceAll("\\s", "");
                        body.put("acceleration", Acceleration.substring(1, Acceleration.length() - 1));
                        String Gyroscope = Arrays.toString(GyroReading).replaceAll("\\s", "");
                        body.put("gyroscope", Gyroscope.substring(1, Gyroscope.length() - 1));
                        String Magenetic = Arrays.toString(MagReading).replaceAll("\\s", "");
                        body.put("magnetic_field", Magenetic.substring(1, Magenetic.length() - 1));
                        // Posting *****************************************************************
                        Log.d("Padmal", body.toString());
                        lastPose = null;
                        return body;
                    } catch (Exception e) {
                        return null;
                    }
                }
            };
            stringRequest.setRetryPolicy(retryPolicy);
            queue.add(stringRequest);
        } catch (Exception e) {
            /**/
        }
    }

    @Override
    public void onSensorChanged(SensorEvent sensorEvent) {
        Sensor sensor = sensorEvent.sensor;

        if (sensor.getType() == Sensor.TYPE_ACCELEROMETER) {
            AccReading[0] = sensorEvent.values[0];
            AccReading[1] = sensorEvent.values[1];
            AccReading[2] = sensorEvent.values[2];
            accX.setText(String.valueOf(AccReading[0]));
            accY.setText(String.valueOf(AccReading[1]));
            accZ.setText(String.valueOf(AccReading[2]));
            mGravity = sensorEvent.values;
        }
        if (sensor.getType() == Sensor.TYPE_GYROSCOPE) {
            GyroReading[0] = sensorEvent.values[0];
            GyroReading[1] = sensorEvent.values[1];
            GyroReading[2] = sensorEvent.values[2];
            gyrX.setText(String.valueOf(GyroReading[0]));
            gyrY.setText(String.valueOf(GyroReading[1]));
            gyrZ.setText(String.valueOf(GyroReading[2]));
        }
        if (sensor.getType() == Sensor.TYPE_MAGNETIC_FIELD) {
            MagReading[0] = sensorEvent.values[0];
            MagReading[1] = sensorEvent.values[1];
            MagReading[2] = sensorEvent.values[2];
            magX.setText(String.valueOf(MagReading[0]));
            magY.setText(String.valueOf(MagReading[1]));
            magZ.setText(String.valueOf(MagReading[2]));
            mGeomagnetic = sensorEvent.values;
        }
        float[] R = new float[9];
        float[] I = new float[9];
        boolean success = SensorManager.getRotationMatrix(R, I, mGravity, mGeomagnetic);
        if (success) {
            float[] orientationData = new float[3];
            SensorManager.getOrientation(R, orientationData);
            OriReading[0] = orientationData[0]; // Azimuth
            OriReading[1] = orientationData[1]; // Pitch
            OriReading[2] = orientationData[2]; // Roll
            oriX.setText(String.valueOf(OriReading[0]));
            oriY.setText(String.valueOf(OriReading[1]));
            oriZ.setText(String.valueOf(OriReading[2]));
        }
    }

    @Override
    public void onAccuracyChanged(Sensor sensor, int i) {

    }

    private double[] generate3x3Matrix(double x, double y, double w, double z) {
        // Calculate intermediate variables
        double wx = w * x;
        double wy = w * y;
        double wz = w * z;
        double xx = x * x;
        double xy = x * y;
        double xz = x * z;
        double yy = y * y;
        double yz = y * z;
        double zz = z * z;
        // Calculate second level values
        double R00 = 1.0 - 2.0 * (yy + zz);
        double R01 = 2.0 * (xy - wz);
        double R02 = 2.0 * (wy + xz);
        // middle row
        double R10 = 2.0 * (xy + wz);
        double R11 = 1.0 - 2.0 * (xx + zz);
        double R12 = 2 * (-wx + yz);
        // bottom row
        double R20 = 2 * (-wy + xz);
        double R21 = 2 * (wx + yz);
        double R22 = 1.0 - 2.0 * (xx + yy);
        // Generate Matrix
        double[][] matrixData = {{R00, R01, R02, x}, {R10, R11, R12, y}, {R20, R21, R22, z}, {0.0, 0.0, 0.0, 1.0}};
        RealMatrix matrix = MatrixUtils.createRealMatrix(matrixData);
        double[][] uwTss = {{1.0, 0.0, 0.0, 0.0}, {0.0, 1.0, 0.0, 0.0}, {0.0, 0.0, -1.0, 0.0}, {0.0, 0.0, 0.0, 1.0}};
        RealMatrix UWTSS = MatrixUtils.createRealMatrix(uwTss);
        double[][] dTuc = {{1.0, 0.0, 0.0, 0.0}, {0.0, 1.0, 0.0, 0.0}, {0.0, 0.0, -1.0, 0.0}, {0.0, 0.0, 0.0, 1.0}};
        RealMatrix DTUC = MatrixUtils.createRealMatrix(dTuc);

        // Multiply first two matrices
        RealMatrix UWTD = UWTSS.multiply(matrix);
        // Multiply that matrix by last matrix
        RealMatrix UWTUC = UWTD.multiply(DTUC);

        //double[][] data = UWTUC.getData();
        double[][] data = matrix.getData();
        StringBuilder matmaker = new StringBuilder();
        for (double[] line : data) {
            for (double l : line) {
                matmaker.append(String.valueOf(l));
                matmaker.append(",");
            }
            matmaker.append("---");
        }
        Log.d("Padmal", matmaker.toString());

        double W, X, Y, Z;
        double dW, dX, dY, dZ;

        dW = 1.0 + data[0][0] + data[1][1] + data[2][2];
        dX = 1.0 + data[0][0] - data[1][1] - data[2][2];
        dY = 1.0 - data[0][0] + data[1][1] - data[2][2];
        dZ = 1.0 - data[0][0] - data[1][1] + data[2][2];

        double MaxDivisor = Math.max(Math.max(Math.max(dW, dX), dY), dZ);

        if (MaxDivisor == dW) {
            W = 0.5 * Math.sqrt(dW);
            X = (data[1][2] - data[2][1]) / (4.0 * W);
            Y = (data[2][0] - data[0][2]) / (4.0 * W);
            Z = (data[0][1] - data[1][0]) / (4.0 * W);
        } else if (MaxDivisor == dX) {
            X = 0.5 * Math.sqrt(dX);
            Y = (data[0][1] + data[1][0]) / (4.0 * X);
            Z = (data[0][2] + data[2][0]) / (4.0 * X);
            W = (data[1][2] - data[2][1]) / (4.0 * X);
        } else if (MaxDivisor == dY) {
            Y = 0.5 * Math.sqrt(dY);
            X = (data[0][1] + data[1][0]) / (4.0 * Y);
            Z = (data[1][2] + data[2][1]) / (4.0 * Y);
            W = (data[2][0] - data[0][2]) / (4.0 * Y);
        } else {
            Z = 0.5 * Math.sqrt(dZ);
            X = (data[0][2] + data[2][0]) / (4.0 * Z);
            Y = (data[1][2] + data[2][1]) / (4.0 * Z);
            W = (data[0][1] + data[1][0]) / (4.0 * Z);
        }

        Log.d("Padmal", " W = " + W + " X = " + X + " Y = " + Y + " Z = " + Z);
        double[] returnDouble = {W, X, Y, Z};
        return returnDouble;
    }

    private double[] newMatrix(double x, double y, double w, double z) {

        double[] tempQuart = new double[4];
        tempQuart[0] = x;
        tempQuart[1] = y;
        tempQuart[2] = z;
        tempQuart[3] = w;

        double[] mat = QuatToMatrix3(tempQuart);
        //rotate 90 degree
        double[] matrixRotate90Degree = rotate90DegreeAlongZ(mat);
        double[] Leftmatrix = transformToLeftCoordinateSystem(matrixRotate90Degree);
        return Matrix4ToEuler(Leftmatrix, new double[]{lastPose.getTranslationAsFloats()[0],
                lastPose.getTranslationAsFloats()[1], lastPose.getTranslationAsFloats()[2]});
    }

    private double[] QuatToMatrix3(double[] quat) {

        double x = quat[0];
        double y = quat[1];
        double z = quat[2];
        double w = quat[3];
        double d2 = x * x + y * y + z * z + w * w;
        double s = 2.0 / d2;
        double xs = x * s, ys = y * s, zs = z * s;
        double wx = w * xs, wy = w * ys, wz = w * zs;
        double xx = x * xs, xy = x * ys, xz = x * zs;
        double yy = y * ys, yz = y * zs, zz = z * zs;

        double[] mat = new double[9];

        mat[0] = 1.0 - (yy + zz);
        mat[1] = xy + wz;
        mat[2] = xz - wy;

        mat[3] = xy - wz;
        mat[4] = 1.0 - (xx + zz);
        mat[5] = yz + wx;

        mat[6] = xz + wy;
        mat[7] = yz - wx;
        mat[8] = 1.0 - (xx + yy);

        return mat;
    }

    private double[] transformToLeftCoordinateSystem(double[] mat) {

        double[] Leftmat = new double[9];

        Leftmat[0] = mat[4];
        Leftmat[1] = -mat[5];
        Leftmat[2] = -mat[3];

        Leftmat[3] = -mat[7];
        Leftmat[4] = mat[8];
        Leftmat[5] = mat[6];

        Leftmat[6] = -mat[1];
        Leftmat[7] = mat[2];
        Leftmat[8] = mat[0];

        return Leftmat;
    }

    private double[] rotate90DegreeAlongZ(double[] mat) {

        double[] rotate90degree = new double[9];

        rotate90degree[0] = mat[1];
        rotate90degree[1] = -mat[0];
        rotate90degree[2] = mat[2];

        rotate90degree[3] = mat[4];
        rotate90degree[4] = -mat[3];
        rotate90degree[5] = mat[5];

        rotate90degree[6] = mat[7];
        rotate90degree[7] = -mat[6];
        rotate90degree[8] = mat[8];

        return rotate90degree;
    }

    private double[] Matrix4ToEuler(double[] mat, double[] oldPose) {

        double _trX, _trY;
        double[] leftEuler = new double[3];

        // Calculate Y-axis angle
        if (mat[0] > 0.0) {
            leftEuler[1] = Math.asin(mat[6]);
        } else {
            leftEuler[1] = Math.PI - Math.asin(mat[6]);
        }

        double C = Math.cos(leftEuler[1]);
        if (Math.abs(C) > 0.005) {                 // Gimball lock?
            _trX = mat[8] / C;             // No, so get X-axis angle
            _trY = -mat[7] / C;
            leftEuler[0] = Math.atan2(_trY, _trX);
            _trX = mat[0] / C;              // Get Z-axis angle
            _trY = -mat[3] / C;
            leftEuler[2] = Math.atan2(_trY, _trX);
        } else {                                    // Gimball lock has occurred
            leftEuler[0] = 0.0;                       // Set X-axis angle to zero
            _trX = mat[4];  //1                // And calculate Z-axis angle
            _trY = mat[1];  //2
            leftEuler[2] = Math.atan2(_trY, _trX);
        }

        double[] newPose = new double[6];
        /*
        newPose[0] = leftEuler[0]; // Rotation_X
        newPose[1] = leftEuler[1]; // Rotation_Y
        newPose[2] = leftEuler[2]; // Rotation_Z
        */
        newPose[3] = leftEuler[0]; // Rotation_X
        newPose[4] = leftEuler[1]; // Rotation_Y
        newPose[5] = leftEuler[2]; // Rotation_Z

        //rotate 90 degree
        double x_90, y_90, z_90;
        x_90 = oldPose[1];
        y_90 = -oldPose[0];
        z_90 = oldPose[2];

        //to left hand coordinate system
        /*
        newPose[3] = -y_90; // New X
        newPose[4] = z_90; // New Y
        newPose[5] = x_90; // New Z
        */
        newPose[0] = -y_90; // New X
        newPose[1] = z_90; // New Y
        newPose[2] = x_90; // New Z
        return newPose;
    }
}
