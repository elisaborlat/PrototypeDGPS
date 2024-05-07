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

import com.google.android.material.bottomnavigation.BottomNavigationView;


import java.util.HashMap;
import java.util.Map;

public class MainActivity extends AppCompatActivity {

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

        // The fragmentManager need to be created juste once
        // Load default Fragment
        FragmentManager fragmentManager = getSupportFragmentManager();
        fragmentManager.beginTransaction()
                .replace(R.id.fragment_container, new HomeFragment(), null)
                .setReorderingAllowed(true)
                .commit();

        // Navigation View
        BottomNavigationView navigationView = findViewById(R.id.bottom_nav);
        Map<Integer, Fragment> fragmentMap = new HashMap<>();
        fragmentMap.put(R.id.navigation_home, new HomeFragment());
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
                .setTitle("Localisation désactivée")
                .setMessage("Pour utiliser cette application, veuillez activer la localisation.")
                .setPositiveButton("Paramètres", (dialog, which) -> {
                    // Open the location settings
                    Intent intent = new Intent(Settings.ACTION_LOCATION_SOURCE_SETTINGS);
                    startActivity(intent);
                })
                .setNegativeButton("Annuler", (dialog, which) -> {
                    // Close the application or perform other necessary actions
                    // finish();
                })
                .show();
    }

}