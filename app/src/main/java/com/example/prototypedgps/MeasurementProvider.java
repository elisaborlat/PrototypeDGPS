package com.example.prototypedgps;


import android.content.Context;
import android.content.pm.PackageManager;
import android.location.GnssMeasurementsEvent;
import android.location.GnssNavigationMessage;
import android.location.GnssStatus;
import android.location.LocationManager;

import androidx.annotation.NonNull;
import androidx.core.app.ActivityCompat;
import androidx.core.content.ContextCompat;

import java.util.Arrays;
import java.util.List;


public class MeasurementProvider {

    public static final String TAG = "MeasurementProvider";
    LocationManager mLocationManager;

    private final Context context;

    private final List<MeasurementListener> mListeners;

    public MeasurementProvider(Context context, MeasurementListener... loggers) {
        this.mListeners = Arrays.asList(loggers);
        mLocationManager = (LocationManager) context.getSystemService(Context.LOCATION_SERVICE);
        this.context = context;
    }

    private final GnssMeasurementsEvent.Callback gnssMeasurementsEventListener =
            new GnssMeasurementsEvent.Callback() {
                @Override
                public void onGnssMeasurementsReceived(GnssMeasurementsEvent event) {
                    for (MeasurementListener logger : mListeners) {
                        logger.onGnssMeasurementsReceived(event);
                    }

                }
            };

    private final GnssNavigationMessage.Callback gnssNavigationMessage =
            new GnssNavigationMessage.Callback() {
                @Override
                public void onGnssNavigationMessageReceived(GnssNavigationMessage navigationMessage) {
                    for (MeasurementListener logger : mListeners) {
                        logger.onGnssNavigationMessageReceived(navigationMessage);
                    }

                }
            };

    private final GnssStatus.Callback gnssStatusListener =
            new GnssStatus.Callback() {
                @Override
                public void onSatelliteStatusChanged(@NonNull GnssStatus status) {
                    for (MeasurementListener logger : mListeners) {
                        logger.onGnssStatusChanged(status);
                    }
                }
            };


    public void registerMeasurements() {
        try {
            mLocationManager.registerGnssMeasurementsCallback(gnssMeasurementsEventListener);
            mLocationManager.registerGnssNavigationMessageCallback(gnssNavigationMessage);
        } catch (SecurityException ignored) {
        }
    }

    public void registerStatus() {
        if (ActivityCompat.checkSelfPermission(context, android.Manifest.permission.ACCESS_FINE_LOCATION) != PackageManager.PERMISSION_GRANTED) {
            // TODO: Consider calling
            //    ActivityCompat#requestPermissions
            // here to request the missing permissions, and then overriding
            //   public void onRequestPermissionsResult(int requestCode, String[] permissions,
            //                                          int[] grantResults)
            // to handle the case where the user grants the permission. See the documentation
            // for ActivityCompat#requestPermissions for more details.
            return;
        }
        mLocationManager.registerGnssStatusCallback(gnssStatusListener);
    }

    public void unregisterStatus(){
        mLocationManager.unregisterGnssStatusCallback(gnssStatusListener);
    }

    public void unregisterMeasurements() {
        mLocationManager.unregisterGnssMeasurementsCallback(gnssMeasurementsEventListener);
        mLocationManager.unregisterGnssNavigationMessageCallback(gnssNavigationMessage);
    }

    public void unregisterAll() {
        unregisterMeasurements();
        unregisterStatus();
    }
}
