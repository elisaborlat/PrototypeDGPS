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

    public StatusFragment(RealTimePositionCalculator mRealTimePositionCalculator) {
        this.mRealTimePositionCalculator = mRealTimePositionCalculator;
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
        public void updateTemp() {
            handler.post(() -> {
                Activity activity = getActivity();
                if (activity != null) {
                    binding.textView18.setText("Elisa Borlat");
                    }
            });
        }

        public void updateStatusTable(ArrayList<Map> allGpsSat){
            handler.post(() -> {
                Activity activity = getActivity();
                if (activity != null) {
                    int rowIndex = 1; // Start after header row
                    for (Map satellite : allGpsSat) {
                        updateOrAddRow(binding.tableLayoutStatus, satellite, rowIndex);
                        rowIndex++;
                    }

                    // Remove extra rows if any
                    /*
                    while (rowIndex < binding.tableLayoutStatus.getChildCount()) {
                        binding.tableLayoutStatus.removeViewAt(rowIndex);
                    }

                    for(Map sat: allGpsSat) {




                        TextView column1 = new TextView(getContext());
                        column1.setText( (String) sat.get("svId"));
                        tableRow.addView(column1);

                        TextView column2 = new TextView(getContext());
                        column2.setText((String) sat.get("cn0DbHz"));
                        tableRow.addView(column2);

                        tableRow.addView(new TextView(getContext()));


                        TextView column4 = new TextView(getContext());
                        column4.setText((String) sat.get("elevation"));
                        tableRow.addView(column4);

                        TextView column5 = new TextView(getContext());
                        column5.setText((String) sat.get("azimuth"));
                        tableRow.addView(column5);

                        binding.tableLayoutStatus.addView(tableRow);
                    }

                    binding.textView18.setText("Elisa Borlat");*/
                }
            });
        }
    }

    private void updateOrAddRow(TableLayout tableLayout, Map satellite, int rowIndex) {
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



    private void generateTable(int numRows, int numCols) {
        for (int row = 0; row < numRows; row++) {
            TableRow tableRow = new TableRow(getContext());

            for (int col = 0; col < numCols; col++) {
                String cellId = "cell_" + row + "_" + col; // Unique ID for the cell
                String cellValue = "Row " + row + ", Col " + col;

                TextView textView = new TextView(getContext());
                textView.setText(cellValue);
                textView.setId(View.generateViewId()); // Unique ID for the TextView

                // Set the cell ID as a tag for the TextView
                //textView.setTag(R.id.cell_id, cellId);

                tableRow.addView(textView);
            }

            //tableLayout.addView(tableRow);
        }
    }

}
