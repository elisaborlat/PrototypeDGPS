package com.example.prototypedgps;

import androidx.annotation.NonNull;
import androidx.appcompat.app.AppCompatActivity;
import androidx.fragment.app.Fragment;
import androidx.fragment.app.FragmentManager;

import android.os.Bundle;
import android.view.MenuItem;

import com.google.android.material.bottomnavigation.BottomNavigationView;
import com.google.android.material.navigation.NavigationBarView;

import java.util.HashMap;
import java.util.Map;

public class MainActivity extends AppCompatActivity {

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

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
        navigationView.setOnItemSelectedListener(new NavigationBarView.OnItemSelectedListener() {
            @Override
            public boolean onNavigationItemSelected(@NonNull MenuItem item) {
                Fragment fragment = fragmentMap.get(item.getItemId());
                if (fragment != null) {
                    fragmentManager.beginTransaction()
                            .replace(R.id.fragment_container, fragment, null)
                            .setReorderingAllowed(true)
                            .commit();
                    return true;
                }
                return false;
            }
        });



    }

}