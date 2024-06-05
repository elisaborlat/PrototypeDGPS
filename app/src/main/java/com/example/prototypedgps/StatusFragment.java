package com.example.prototypedgps;

import android.app.Activity;

import android.os.Bundle;
import android.os.Handler;
import android.os.Looper;

import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.TableRow;
import android.widget.TextView;

import androidx.annotation.NonNull;
import androidx.fragment.app.Fragment;

import com.example.prototypedgps.databinding.FragmentStatusBinding;

public class StatusFragment extends Fragment {

    private FragmentStatusBinding binding;

    private final Handler handler = new Handler(Looper.getMainLooper());

    private RealTimePositionCalculator mRealTimePositionCalculator;

    private final StatusUIFragmentComponent mStatusUIFragmentComponent = new StatusUIFragmentComponent();

    public StatusFragment(RealTimePositionCalculator mRealTimePositionCalculator) {
        this.mRealTimePositionCalculator = mRealTimePositionCalculator;
    }


    @Override
    public View onCreateView(@NonNull LayoutInflater inflater,
                             ViewGroup container,
                             Bundle savedInstanceState) {
        binding = FragmentStatusBinding.inflate(inflater, container, false);

        TableRow tableRow = new TableRow(getContext());

        TextView column1 = new TextView(getContext());
        TextView column2 = new TextView(getContext());

        column1.setText("mGnssStatus.getSatelliteCount()");
        column2.setText("34.7");
        tableRow.addView(column1);
        tableRow.addView(column2);

        binding.tableLayoutStatus.addView(tableRow);

        if (mRealTimePositionCalculator != null) {
            mRealTimePositionCalculator.setStatusUiFragmentComponent(mStatusUIFragmentComponent);
        }

        return binding.getRoot();
    }

    public void setRealTimePositionCalculator(RealTimePositionCalculator mRealTimePositionCalculator) {
        this.mRealTimePositionCalculator = mRealTimePositionCalculator;
    }

    @Override
    public void onDestroyView() {
        super.onDestroyView();
        binding = null;
    }


    public class StatusUIFragmentComponent {
        public void updateTemp() {
            handler.post(() -> {
                Activity activity = getActivity();
                if (activity != null) {
                    binding.textView18.setText("Elisa Borlat");
                    }
            });
        }

        public void updateStatusTable(){
            handler.post(() -> {
                Activity activity = getActivity();
                if (activity != null) {
                    binding.textView18.setText("Elisa Borlat");
                }
            });
        }
    }
}
