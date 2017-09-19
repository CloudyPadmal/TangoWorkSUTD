package com.projecttango.examples.java.pointcloud;

import android.content.Context;
import android.util.AttributeSet;
import android.widget.TextView;

/**
 * Created by padmal on 9/20/17.
 */

public class CircleTextView extends android.support.v7.widget.AppCompatTextView {

    public CircleTextView(Context context) {
        super(context);
    }

    public CircleTextView(Context context, AttributeSet attrs) {
        super(context, attrs);
    }

    public CircleTextView(Context context, AttributeSet attrs, int defStyleAttr) {
        super(context, attrs, defStyleAttr);
    }

    @Override
    protected void onMeasure(int widthMeasureSpec, int heightMeasureSpec) {
        super.onMeasure(widthMeasureSpec, heightMeasureSpec);
        int width = getMeasuredWidth();
        setMeasuredDimension(width, width);
    }
}
