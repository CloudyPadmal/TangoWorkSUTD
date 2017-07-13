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
import android.annotation.TargetApi;
import android.app.Activity;
import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.content.SharedPreferences;
import android.content.pm.PackageManager;
import android.graphics.Color;
import android.hardware.display.DisplayManager;
import android.net.wifi.ScanResult;
import android.net.wifi.WifiManager;
import android.os.Build;
import android.os.Bundle;
import android.os.Handler;
import android.util.Log;
import android.util.StringBuilderPrinter;
import android.view.Display;
import android.view.MotionEvent;
import android.view.View;
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

import org.json.JSONArray;
import org.json.JSONObject;
import org.rajawali3d.scene.ASceneFrameCallback;
import org.rajawali3d.surface.RajawaliSurfaceView;
import org.w3c.dom.Node;

import java.nio.FloatBuffer;
import java.util.ArrayList;
import java.util.Calendar;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Timer;
import java.util.TimerTask;

public class PointCloudActivity extends Activity {
    private static final String TAG = PointCloudActivity.class.getSimpleName();

    private static final String UX_EXCEPTION_EVENT_DETECTED = "Exception Detected: ";
    private static final String UX_EXCEPTION_EVENT_RESOLVED = "Exception Resolved: ";

    private static final int SECS_TO_MILLISECS = 1000;
    private static final double UPDATE_INTERVAL_MS = 200.0;

    private Tango mTango;
    private TangoConfig mConfig;
    private TangoUx mTangoUx;

    private TangoPointCloudManager mPointCloudManager;
    private PointCloudRajawaliRenderer mRenderer;
    private RajawaliSurfaceView mSurfaceView;
    private TextView mPointCountTextView;

    private TextView mAverageZTextView;
    private double mPointCloudPreviousTimeStamp;

    private boolean mIsConnected = false;

    private double mPointCloudTimeToNextUpdate = UPDATE_INTERVAL_MS;

    private int mDisplayRotation = 0;

    // Custom variables
    private TextView mTime;
    private TextView mNodes;
    private TextView mCurrently;
    private Switch modeSwitch;

    private Timer timer;

    private final String CLOUD_URL = "http://202.94.70.33/tango/insert_tango_point_cloud.php";
    private final String WIFI_URL = "http://202.94.70.33/tango/insert_tango_wifi_scan.php";
    private final String MODE = "Mode";

    private TangoPoseData lastPose;
    private TangoPointCloudData lastCloud;

    private SharedPreferences logger;

    private WifiManager wifiManager;
    private List<ScanResult> scanResults;

    private final long CLOUD_INTERVAL = 1000;
    private final long WIFI_INTERVAL = 5000;
    private final int MM = 10000;

    private DefaultRetryPolicy retryPolicy;
    private RequestQueue queue;

    private int NodeCount = 0;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_point_cloud);
        Ask.on(this).forPermissions(Manifest.permission.ACCESS_COARSE_LOCATION,
                Manifest.permission.ACCESS_FINE_LOCATION,
                Manifest.permission.ACCESS_WIFI_STATE,
                Manifest.permission.CHANGE_WIFI_STATE).go();

        mPointCountTextView = (TextView) findViewById(R.id.point_count_textview);
        mAverageZTextView = (TextView) findViewById(R.id.average_z_textview);
        mTime = (TextView) findViewById(R.id.average_time);
        mNodes = (TextView) findViewById(R.id.nodes);
        mCurrently = (TextView) findViewById(R.id.currently);
        modeSwitch = (Switch) findViewById(R.id.wifi_or_pointcloud);
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
        boolean currentState = logger.getBoolean(MODE, false);
        modeSwitch.setChecked(currentState);
        mCurrently.setText(currentState ? "WiFi" : "Cloud");
        if (!currentState) {
            timer.schedule(new TimerTask() {
                @Override
                public void run() {
                    postCloudData();
                }
            }, CLOUD_INTERVAL, CLOUD_INTERVAL);
        } else {
            timer.schedule(new TimerTask() {
                @Override
                public void run() {
                    postWiFiData();
                }
            }, WIFI_INTERVAL, WIFI_INTERVAL);
        }

        modeSwitch.setOnCheckedChangeListener(new CompoundButton.OnCheckedChangeListener() {
            @Override
            public void onCheckedChanged(CompoundButton buttonView, boolean isChecked) {
                logger.edit().putBoolean(MODE, isChecked).apply();
                Toast.makeText(getApplicationContext(), "Restart!", Toast.LENGTH_LONG).show();
            }
        });
    }

    @Override
    protected void onStart() {
        super.onStart();

        mTangoUx.start();
        bindTangoService();
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
                            mAverageZTextView.setText("Waiting");
                            mAverageZTextView.setBackgroundColor(Color.TRANSPARENT);
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
                    String displayText = (response.contains("Data successfully created")) ? "Success" : "Failure";
                    mAverageZTextView.setText(displayText);
                    mAverageZTextView.setBackgroundColor(displayText.contains("Success") ? Color.GREEN : Color.RED);
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
                        body.put("pose",
                                String.valueOf(lastPose.getTranslationAsFloats()[0]) + "," +
                                        String.valueOf(lastPose.getTranslationAsFloats()[1]) + "," +
                                        String.valueOf(lastPose.getTranslationAsFloats()[2]) + "," +
                                        String.valueOf(lastPose.getRotationAsFloats()[0]) + "," +
                                        String.valueOf(lastPose.getRotationAsFloats()[1]) + "," +
                                        String.valueOf(lastPose.getRotationAsFloats()[2]) + "," +
                                        String.valueOf(lastPose.getRotationAsFloats()[3])
                        );
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
                    String displayText = (response.contains("Data successfully created")) ? "Success" : "Failure";
                    mAverageZTextView.setText(displayText);
                    mAverageZTextView.setBackgroundColor(displayText.contains("Success") ? Color.GREEN : Color.RED);
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
                        body.put("pose",
                                String.valueOf(lastPose.getTranslationAsFloats()[0]) + "," +
                                        String.valueOf(lastPose.getTranslationAsFloats()[1]) + "," +
                                        String.valueOf(lastPose.getTranslationAsFloats()[2]) + "," +
                                        String.valueOf(lastPose.getRotationAsFloats()[0]) + "," +
                                        String.valueOf(lastPose.getRotationAsFloats()[1]) + "," +
                                        String.valueOf(lastPose.getRotationAsFloats()[2]) + "," +
                                        String.valueOf(lastPose.getRotationAsFloats()[3])
                        );
                        // Tango Time **************************************************************
                        body.put("tango_time", String.valueOf(System.currentTimeMillis()));
                        // Point Cloud *************************************************************
                        scanResults = wifiManager.getScanResults();
                        int APs = scanResults.size();
                        NodeCount = APs;
                        StringBuilder wifiString = new StringBuilder();
                        wifiString.append("APs:");
                        wifiString.append(APs);
                        wifiString.append(",");
                        JSONArray scanResultsArray = new JSONArray();
                        for (ScanResult result : scanResults) {
                            JSONObject Result = new JSONObject();
                            Result.put("ID", scanResults.indexOf(result) + 1);
                            Result.put("SSID", result.SSID);
                            Result.put("Channel", result.frequency);
                            Result.put("Signal", result.level + " dB");
                            Result.put("BSSID", result.BSSID);
                            scanResultsArray.put(Result);
                        }
                        wifiString.append(scanResultsArray.toString());
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
}
