package com.example.prototypedgps;


import androidx.appcompat.app.AppCompatActivity;
import androidx.core.app.ActivityCompat;
import androidx.core.content.ContextCompat;
import androidx.fragment.app.Fragment;
import androidx.fragment.app.FragmentManager;

import android.Manifest;
import android.app.Activity;
import android.app.AlertDialog;
import android.content.Context;
import android.content.Intent;
import android.content.pm.PackageManager;
import android.location.LocationManager;
import android.os.Bundle;
import android.provider.Settings;

import com.example.pseudorange.EphemerisManager;
import com.google.android.material.bottomnavigation.BottomNavigationView;


import java.io.BufferedReader;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStreamReader;
import java.util.HashMap;
import java.util.Map;

public class MainActivity extends AppCompatActivity{

    private MeasurementProvider mMeasurementProvider;

    // Init EphemerisManager
    private final EphemerisManager mEphemerisManager = new EphemerisManager();

    private static final int LOCATION_REQUEST_ID = 1;

    private static final String[] REQUIRED_PERMISSIONS = {
            android.Manifest.permission.ACCESS_FINE_LOCATION, Manifest.permission.WRITE_EXTERNAL_STORAGE
    };

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        // Check permission
        if (!hasPermissions(this)) {
            ActivityCompat.requestPermissions(this, REQUIRED_PERMISSIONS, LOCATION_REQUEST_ID);
        }

        // Check whether the location service is activated
        if (!isLocationEnabled()) {
            // Display a pop-up window to inform the user
            showLocationDisabledAlert();
        }

        // Init BaseStation
        BaseStation mBaseStation = new BaseStation();
        mBaseStation.registerRtcmMsg();

        // Init RealTimePositionCalculator
        RealTimePositionCalculator mRealTimePositionCalculator = new RealTimePositionCalculator(mBaseStation, mEphemerisManager);

        FileLogger mFileLogger = new FileLogger(this);
        RinexLogger mRinexLogger = new RinexLogger(this);

        // Init MeasurementProvider
        mMeasurementProvider = new MeasurementProvider(
                getApplicationContext(),
                mFileLogger,
                mRinexLogger,
                mRealTimePositionCalculator);

        mMeasurementProvider.registerMeasurements();
        mMeasurementProvider.registerStatus();

        //Set HomeFragment
        HomeFragment homeFragment = new HomeFragment();
        homeFragment.setFileLogger(mFileLogger);
        homeFragment.setRinexLogger(mRinexLogger);
        homeFragment.setRealTimePositionCalculator(mRealTimePositionCalculator);
        homeFragment.setBaseStation(mBaseStation);


        // The fragmentManager need to be created juste once
        FragmentManager fragmentManager = getSupportFragmentManager();
        // Load default Fragment
        fragmentManager.beginTransaction()
                .replace(R.id.fragment_container, homeFragment, null)
                .setReorderingAllowed(true)
                .commit();

        // Navigation View
        BottomNavigationView navigationView = findViewById(R.id.bottom_nav);
        Map<Integer, Fragment> fragmentMap = new HashMap<>();
        fragmentMap.put(R.id.navigation_home, homeFragment);
        fragmentMap.put(R.id.navigation_status, new StatusFragment());
        fragmentMap.put(R.id.navigation_notifications, new BaseFragment());
        navigationView.setOnItemSelectedListener(item -> {
            Fragment fragment = fragmentMap.get(item.getItemId());
            if (fragment != null) {
                fragmentManager.beginTransaction()
                        .replace(R.id.fragment_container, fragment, null)
                        .setReorderingAllowed(true)
                        .commit();
                return true;
            }
            return false;
        });

        // Load last ephemeris
        FileInputStream fis;
        try {
            fis = openFileInput("data.txt");
            BufferedReader reader = new BufferedReader(new InputStreamReader(fis));
            StringBuilder sb = new StringBuilder();
            String line;
            while ((line = reader.readLine()) != null) {
                sb.append(line).append("\n");
            }
            mEphemerisManager.loadLastEph(sb);
            System.out.println("Data have been load : data.txt");
            fis.close();
        } catch (FileNotFoundException e) {
            System.out.println("File data.txt does not exist because it is the first execution of the app.");
        } catch (IOException e) {
            throw new RuntimeException(e);
        }

    }

    @Override
    protected void onResume() {
        super.onResume();

    }

    @Override
    protected void onPause() {
        super.onPause();
        // Storage of current ephemeris
        try {
            FileOutputStream fos = openFileOutput("data.txt", Context.MODE_PRIVATE);
            String s = mEphemerisManager.saveDataToFile(fos);
            System.out.println("onPause| Data have been storage : data.txt");
        } catch (IOException e) {
            throw new RuntimeException(e);
        }


    }

    @Override
    protected void onDestroy() {
        mMeasurementProvider.unregisterAll();
        super.onDestroy();
    }

    private boolean hasPermissions(Activity activity) {
        for (String p : REQUIRED_PERMISSIONS) {
            if (ContextCompat.checkSelfPermission(activity, p) != PackageManager.PERMISSION_GRANTED) {
                return false;
            }
        }
        return true;
    }

    // Check whether the location service is activated
    private boolean isLocationEnabled() {
        LocationManager locationManager = (LocationManager) getSystemService(Context.LOCATION_SERVICE);
        return locationManager != null && locationManager.isProviderEnabled(LocationManager.GPS_PROVIDER);
    }

    // Display a pop-up window to inform the user
    private void showLocationDisabledAlert() {
        new AlertDialog.Builder(this)
                .setTitle("Deactivated location")
                .setMessage("To use this application, please activate localisation.")
                .setPositiveButton("Settings", (dialog, which) -> {
                    // Open the location settings
                    Intent intent = new Intent(Settings.ACTION_LOCATION_SOURCE_SETTINGS);
                    startActivity(intent);
                })
                .setNegativeButton("Cancel", (dialog, which) -> {
                    // Close the application or perform other necessary actions
                    // finish();
                })
                .show();
    }
}