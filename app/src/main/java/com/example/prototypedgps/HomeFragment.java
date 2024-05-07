package com.example.prototypedgps;

import android.os.Bundle;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;


import androidx.annotation.NonNull;

import androidx.fragment.app.Fragment;

import com.example.prototypedgps.databinding.FragmentMainBinding;

import java.util.stream.Stream;


public class HomeFragment extends Fragment {

    private FragmentMainBinding binding;


    @Override
    public View onCreateView (@NonNull LayoutInflater inflater,
                              ViewGroup container,
                              Bundle savedInstanceState) {
        binding = FragmentMainBinding.inflate(inflater, container, false);


        binding.buttonStartLog.setOnClickListener(v -> {
            // Handle button click logic when enabled
            binding.buttonStartLog.setEnabled(false);
            binding.buttonStopSend.setEnabled(true);
        });


        binding.buttonStopSend.setOnClickListener(v -> {
            binding.buttonStopSend.setEnabled(false);
            boolean anySwitchOn = Stream.of(binding.switch1.isChecked(), binding.switch2.isChecked(), binding.switch4.isChecked()).anyMatch(Boolean::booleanValue);
            binding.buttonStartLog.setEnabled(anySwitchOn);
        }
        );

        binding.switch1.setOnCheckedChangeListener((buttonView, isChecked) -> {
            if (isChecked) {
                binding.buttonStartLog.setEnabled(true);
            } else {
                if (!binding.switch2.isChecked() && !binding.switch4.isChecked()){
                    binding.buttonStartLog.setEnabled(false);
                }
            }
        });

        binding.switch2.setOnCheckedChangeListener((buttonView, isChecked) -> {
            if (isChecked) {
                binding.buttonStartLog.setEnabled(true);
            } else {
                if (!binding.switch1.isChecked() && !binding.switch4.isChecked()){
                    binding.buttonStartLog.setEnabled(false);
                }
            }
        });

        binding.switch4.setOnCheckedChangeListener((buttonView, isChecked) -> {
            if (isChecked) {
                binding.buttonStartLog.setEnabled(true);
            } else {
                if (!binding.switch1.isChecked() && !binding.switch2.isChecked()){
                    binding.buttonStartLog.setEnabled(false);
                }

            }
        });

        return binding.getRoot();
    }

    @Override
    public void onDestroyView() {
        super.onDestroyView();
        binding = null;
    }
}
