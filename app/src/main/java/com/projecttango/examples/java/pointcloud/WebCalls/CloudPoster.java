package com.projecttango.examples.java.pointcloud.WebCalls;

import android.content.Context;

import com.android.volley.Request;
import com.android.volley.RequestQueue;
import com.android.volley.toolbox.Volley;

/**
 * Created by Padmal on 30/6/2017.
 */

public class CloudPoster {

    private static CloudPoster mAppSingletonInstance;
    private RequestQueue mRequestQueue;
    private static Context mContext;

    private CloudPoster(Context context) {
        mContext = context;
        mRequestQueue = getRequestQueue();
    }

    public static synchronized CloudPoster getInstance(Context context) {
        if (mAppSingletonInstance == null) {
            mAppSingletonInstance = new CloudPoster(context);
        }
        return mAppSingletonInstance;
    }

    public RequestQueue getRequestQueue() {
        if (mRequestQueue == null) {
            // getApplicationContext() is key, it keeps you from leaking the
            // Activity or BroadcastReceiver if someone passes one in.
            mRequestQueue = Volley.newRequestQueue(mContext.getApplicationContext());
        }
        return mRequestQueue;
    }

    public <T> void addToRequestQueue(Request<T> req, String tag) {
        req.setTag(tag);
        getRequestQueue().add(req);
    }

    public void cancelPendingRequests(Object tag) {
        if (mRequestQueue != null) {
            mRequestQueue.cancelAll(tag);
        }
    }
}
