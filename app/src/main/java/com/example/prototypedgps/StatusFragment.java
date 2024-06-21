package com.example.prototypedgps;

import android.app.Activity;

import android.os.Bundle;
import android.os.Handler;
import android.os.Looper;

import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.TableLayout;
import android.widget.TableRow;
import android.widget.TextView;

import androidx.annotation.NonNull;
import androidx.fragment.app.Fragment;

import com.example.prototypedgps.databinding.FragmentStatusBinding;

import java.util.ArrayList;
import java.util.Map;

public class StatusFragment extends Fragment {

    private FragmentStatusBinding binding;

    private final Handler handler = new Handler(Looper.getMainLooper());

    private RealTimePositionCalculator mRealTimePositionCalculator;

    private final StatusUIFragmentComponent mStatusUIFragmentComponent = new StatusUIFragmentComponent();

    public StatusFragment() {
    }

    public void setRealTimePositionCalculator(RealTimePositionCalculator realTimePositionCalculator){
        this.mRealTimePositionCalculator = realTimePositionCalculator;
    }



    @Override
    public View onCreateView(@NonNull LayoutInflater inflater,
                             ViewGroup container,
                             Bundle savedInstanceState) {
        binding = FragmentStatusBinding.inflate(inflater, container, false);



        if (mRealTimePositionCalculator != null) {
            mRealTimePositionCalculator.setStatusUiFragmentComponent(mStatusUIFragmentComponent);
        }

        return binding.getRoot();
    }

    @Override
    public void onDestroyView() {
        super.onDestroyView();
        binding = null;
    }


    public class StatusUIFragmentComponent {

        public void updateStatusTable(ArrayList<Map<String,String>> allGpsSat){
            handler.post(() -> {
                Activity activity = getActivity();
                if (activity != null) {
                    int rowIndex = 1; // Start after header row
                    for (Map<String,String> satellite : allGpsSat) {
                        updateOrAddRow(binding.tableLayoutStatus, satellite, rowIndex);
                        rowIndex++;
                    }
                }
            });
        }
    }

    private void updateOrAddRow(TableLayout tableLayout, Map<String,String> satellite, int rowIndex) {
        TableRow tableRow;
        if (rowIndex < tableLayout.getChildCount()) {
            // Update existing row
            tableRow = (TableRow) tableLayout.getChildAt(rowIndex);
            updateCell((TextView) tableRow.getChildAt(0), (String) satellite.get("svId"));
            updateCell((TextView) tableRow.getChildAt(1), (String)  satellite.get("cn0DbHz"));
            updateCell((TextView) tableRow.getChildAt(2), "U");
            updateCell((TextView) tableRow.getChildAt(3), (String) satellite.get("elevation"));
            updateCell((TextView) tableRow.getChildAt(4), (String) satellite.get("azimuth"));
        } else {
            // Add new row
            tableRow = new TableRow(getContext());
            tableRow.addView(createTextView((String) satellite.get("svId")));
            tableRow.addView(createTextView((String)  satellite.get("cn0DbHz")));
            tableRow.addView(createTextView("U"));
            tableRow.addView(createTextView((String) satellite.get("elevation")));
            tableRow.addView(createTextView((String) satellite.get("azimuth")));
            tableLayout.addView(tableRow);
        }
    }

    private void updateCell(TextView cell, String newText) {
        cell.setText(newText);
    }

    private TextView createTextView(String text) {
        TextView textView = new TextView(getContext());
        textView.setText(text);
        textView.setPadding(8, 8, 8, 8);
        return textView;
    }

}