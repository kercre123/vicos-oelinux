/*====*====*====*====*====*====*====*====*====*====*====*====*====*====*====*
  Copyright (c) 2015 - 2017 Qualcomm Technologies, Inc.
  All Rights Reserved.
  Confidential and Proprietary - Qualcomm Technologies, Inc.
=============================================================================*/

package my.tests.snapdragonsdktest;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.LinkedHashMap;

import android.app.Activity;
import android.app.PendingIntent;
import android.app.AlertDialog;
import android.content.Intent;
import android.content.DialogInterface;
import android.os.Bundle;
import android.util.Log;

import android.widget.Button;
import android.widget.TextView;
import android.widget.ArrayAdapter;
import android.widget.EditText;
import android.widget.ListView;
import android.widget.AdapterView;
import android.view.View;
import android.view.LayoutInflater;
import android.graphics.Color;
import android.location.Location;

import com.qti.location.sdk.IZatManager;
import com.qti.location.sdk.IZatFlpService;
import com.qti.location.sdk.IZatGeofenceService;

import android.os.Parcel;
import android.os.Parcelable;
import android.os.BadParcelableException;


public class FullscreenActivity extends Activity {

    private static String TAG = "GeofenceTest";
    private static final boolean VERBOSE = Log.isLoggable(TAG, Log.VERBOSE);
    private IZatGeofenceService.IZatGeofenceHandle mHandle = null;
    private IZatManager mIzatMgr;
    private IZatGeofenceService mGfService;
    int mGeofenceSelectedIdx = -1;

    Map<IZatGeofenceService.IZatGeofenceHandle,
            IZatGeofenceService.IzatGeofence>  mGeofenceHandleDataMap =
            new LinkedHashMap<IZatGeofenceService.IZatGeofenceHandle, IZatGeofenceService.IzatGeofence> ();
    ArrayList<String> mGeofenceListNames = new ArrayList<String>();
    ArrayAdapter<String> mGeofenceListadapter;


    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_fullscreen);
        doInit();
    }

    @Override
    protected void onNewIntent(Intent intent) {
        if (intent != null) {
            int transition;
            Location location;
            String strCtx = intent.getStringExtra("context-data");
            if (strCtx != null) {
                Log.v(TAG, "Context:" + strCtx);
            } else {
                Bundle bundleObj = intent.getBundleExtra("context-data");
                if (bundleObj != null) {
                    bundleObj.setClassLoader(TestParcelable.class.getClassLoader());
                    TestParcelable testObj = bundleObj.getParcelable("bundle-obj");
                    if (testObj != null) {
                        if (VERBOSE) {
                            Log.v(TAG, "Context: i = " + testObj.mitest + " s = " + testObj.mstest);
                        }
                    }
                }
            }
            location = intent.getParcelableExtra("transition-location");
            transition = intent.getIntExtra("transition-event", 0);

            if (location != null) {
                Log.v(TAG, "transition:" + transition + " location:" +
                location.toString());
            }
        }
    }

    private void doInit() {
        mIzatMgr = IZatManager.getInstance(getApplicationContext());
        mGfService = mIzatMgr.connectGeofenceService();
        final BreachCallback breachCb = new BreachCallback();

        String version = mIzatMgr.getVersion();
        if (VERBOSE) {
            Log.v(TAG, "SDK and Service Version:" + version);
        }

        // register callback
        mGfService.registerForGeofenceCallbacks(breachCb);


        // register pending intent for this activity
        Intent activitIntent =
                new Intent(getApplicationContext(), FullscreenActivity.class);
        PendingIntent geofenceIntent = PendingIntent.getActivity(
                getApplicationContext(), 0, activitIntent, 0);
        mGfService.registerPendingIntent(geofenceIntent);

        final Button addGeofence = (Button) findViewById(R.id.addBtn);
        final Button editGeofence = (Button) findViewById(R.id.editBtn);
        final Button delGeofence = (Button) findViewById(R.id.delBtn);
        final Button pauseGeofence = (Button) findViewById(R.id.pauseBtn);
        final Button resumeGeofence = (Button) findViewById(R.id.resumeBtn);
        final Button recoverAllGeofence = (Button) findViewById(R.id.recoverAllBtn);

        final ListView lstView = (ListView) findViewById(R.id.geofenceList);
        mGeofenceListadapter = new ArrayAdapter<String>(
                this, android.R.layout.simple_list_item_1, mGeofenceListNames);
        lstView.setAdapter(mGeofenceListadapter);

        lstView.setOnItemClickListener(new AdapterView.OnItemClickListener() {
            @Override
            public void onItemClick(AdapterView<?> a, View v, int position,
                    long id) {
                mGeofenceSelectedIdx = (int)id;
            }
        });

        addGeofence.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                showAddDialog();
            }
        });

        editGeofence.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                if (mGeofenceSelectedIdx != -1) {
                    showEditDialog();
                }
            }
        });

        delGeofence.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                if (mGeofenceSelectedIdx != -1) {
                    Object gfHandleArray[] =
                            mGeofenceHandleDataMap.keySet().toArray();
                    IZatGeofenceService.IZatGeofenceHandle gfHandle =
                            (IZatGeofenceService.IZatGeofenceHandle) gfHandleArray[mGeofenceSelectedIdx];

                    if (gfHandle != null) {
                        mGfService.removeGeofence(gfHandle);
                        mGeofenceHandleDataMap.remove(gfHandle);
                        mGeofenceListNames.remove(mGeofenceSelectedIdx);
                        mGeofenceListadapter.notifyDataSetChanged();
                    }
                }
            }
        });

        pauseGeofence.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                if (mGeofenceSelectedIdx != -1) {
                    Object gfHandleArray[] =
                            mGeofenceHandleDataMap.keySet().toArray();
                    IZatGeofenceService.IZatGeofenceHandle gfHandle =
                            (IZatGeofenceService.IZatGeofenceHandle) gfHandleArray[mGeofenceSelectedIdx];

                    if (gfHandle != null) {
                        gfHandle.pause();
                    }
                }
            }
        });

        resumeGeofence.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                if (mGeofenceSelectedIdx != -1) {
                    showResumeDialog();
                }
            }
        });

        recoverAllGeofence.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                mGeofenceHandleDataMap.clear();
                mGeofenceListNames.clear();

                mGeofenceHandleDataMap.putAll(mGfService.recoverGeofences());
                for (Map.Entry<IZatGeofenceService.IZatGeofenceHandle, IZatGeofenceService.IzatGeofence>
                        entry : mGeofenceHandleDataMap.entrySet()) {
                    IZatGeofenceService.IZatGeofenceHandle gfHandle = entry.getKey();
                    Object geofenceCtx = gfHandle.getContext();
                    if ((geofenceCtx != null) && (geofenceCtx instanceof String)) {
                        mGeofenceListNames.add(geofenceCtx.toString());
                    } else if ((geofenceCtx != null) && (geofenceCtx instanceof Bundle)) {
                        Bundle bundleObj = (Bundle) geofenceCtx;
                        bundleObj.setClassLoader(TestParcelable.class.getClassLoader());
                        TestParcelable testObj = bundleObj.getParcelable("bundle-obj");
                        if (testObj != null) {
                            if (VERBOSE) {
                                Log.v(TAG, "i = " + testObj.mitest + " s = " + testObj.mstest);
                            }
                        }
                        mGeofenceListNames.add("<<no name>>");
                    }
                }

                mGeofenceListadapter.notifyDataSetChanged();
           }
        });
    }

    private void showResumeDialog() {
        LayoutInflater inflater;
        View promptsView;
        EditText editName, editLatitude, editLongitude, editRadius, editTransition;
        EditText editDwellType, editDwellTime, editConfidence, editResponsiveness;

        // get the prompts view
        inflater = LayoutInflater.from(this);
        promptsView = inflater.inflate(R.layout.prompts, null);

        editName  = (EditText)(promptsView.findViewById(R.id.editAlertName));
        editLatitude = (EditText)(promptsView.findViewById(R.id.editTextLatitude));
        editLongitude = (EditText)(promptsView.findViewById(R.id.editTextLongitude));
        editRadius = (EditText)(promptsView.findViewById(R.id.editTextRadius));
        editTransition = (EditText)(promptsView.findViewById(R.id.editTextTransition));
        editDwellType = (EditText)(promptsView.findViewById(R.id.editTextDwellType));
        editDwellTime = (EditText)(promptsView.findViewById(R.id.editDwellTime));
        editConfidence = (EditText)(promptsView.findViewById(R.id.editConfidence));
        editResponsiveness = (EditText)(promptsView.findViewById(R.id.editResponsiveness));

        AlertDialog.Builder alertDialogBuilder =
                new AlertDialog.Builder(this);

        // set prompts.xml to alertdialog builder
        alertDialogBuilder.setView(promptsView);


        // set dialog message
        alertDialogBuilder.setCancelable(false);

        Object gfHandleArray[] = mGeofenceHandleDataMap.keySet().toArray();
        final IZatGeofenceService.IZatGeofenceHandle gfHandle =
                (IZatGeofenceService.IZatGeofenceHandle) gfHandleArray[mGeofenceSelectedIdx];

        if (gfHandle != null) {
            IZatGeofenceService.IzatGeofence gf = mGeofenceHandleDataMap.get(gfHandle);
            if (gf != null) {
                Object geofenceName = gfHandle.getContext();
                if ((geofenceName != null) && (geofenceName instanceof String)) {
                    editName.setText(geofenceName.toString());
                } else {
                    editName.setText("<<no name>>");
                }

                editLatitude.setText(Double.toString(gf.getLatitude()));
                editLongitude.setText(Double.toString(gf.getLongitude()));
                editRadius.setText(Double.toString(gf.getRadius()));
                editTransition.setText(Integer.toString(gf.getTransitionTypes().getValue()));

                IZatGeofenceService.IzatDwellNotify dwellNotify = gf.getDwellNotify();
                editDwellType.setText(Integer.toString(dwellNotify.getDwellType()));
                editDwellTime.setText(Integer.toString(dwellNotify.getDwellTime()));

                editConfidence.setText(Integer.toString(gf.getConfidence().getValue()));
                editResponsiveness.setText(Integer.toString(gf.getNotifyResponsiveness()));
            }
        }
        alertDialogBuilder.setPositiveButton("Resume",
            new DialogInterface.OnClickListener() {
                public void onClick(DialogInterface dialog,int id) {
                    gfHandle.resume();
                }
            });

        alertDialogBuilder.setNegativeButton("Cancel",
            new DialogInterface.OnClickListener() {
                public void onClick(DialogInterface dialog,int id) {
                    dialog.cancel();
                }
            });

        // disable the non-editable fields and just display the current value of all fields
        editName.setFocusable(false);
        editLatitude.setFocusable(false);
        editLongitude.setFocusable(false);
        editRadius.setFocusable(false);
        editDwellType.setFocusable(false);
        editDwellTime.setFocusable(false);
        editConfidence.setFocusable(false);
        editTransition.setFocusable(false);
        editResponsiveness.setFocusable(false);

        editName.setTextColor(Color.GRAY);
        editLatitude.setTextColor(Color.GRAY);
        editLongitude.setTextColor(Color.GRAY);
        editRadius.setTextColor(Color.GRAY);
        editDwellType.setTextColor(Color.GRAY);
        editDwellTime.setTextColor(Color.GRAY);
        editConfidence.setTextColor(Color.GRAY);
        editTransition.setTextColor(Color.GRAY);
        editResponsiveness.setTextColor(Color.GRAY);

        // create alert dialog
        AlertDialog alertDialog = alertDialogBuilder.create();

        // show it
        alertDialog.show();
    }

    private void showEditDialog() {
        LayoutInflater inflater;
        View promptsView;

        // get the prompts view
        inflater = LayoutInflater.from(this);
        promptsView = inflater.inflate(R.layout.prompts, null);

        final EditText editName  = (EditText)(promptsView.findViewById(R.id.editAlertName));
        final EditText editLatitude = (EditText)(promptsView.findViewById(R.id.editTextLatitude));
        final EditText editLongitude = (EditText)(promptsView.findViewById(R.id.editTextLongitude));
        final EditText editRadius = (EditText)(promptsView.findViewById(R.id.editTextRadius));
        final EditText editTransition = (EditText)(promptsView.findViewById(R.id.editTextTransition));
        final EditText editDwellType = (EditText)(promptsView.findViewById(R.id.editTextDwellType));
        final EditText editDwellTime = (EditText)(promptsView.findViewById(R.id.editDwellTime));
        final EditText editConfidence = (EditText)(promptsView.findViewById(R.id.editConfidence));
        final EditText editResponsiveness = (EditText)(promptsView.findViewById(R.id.editResponsiveness));

        AlertDialog.Builder alertDialogBuilder =
                new AlertDialog.Builder(this);

        // set prompts.xml to alertdialog builder
        alertDialogBuilder.setView(promptsView);

        // set dialog message
        alertDialogBuilder.setCancelable(false);

        Object gfHandleArray[] = mGeofenceHandleDataMap.keySet().toArray();
        final IZatGeofenceService.IZatGeofenceHandle gfHandle =
                (IZatGeofenceService.IZatGeofenceHandle) gfHandleArray[mGeofenceSelectedIdx];

        if (gfHandle != null) {
            IZatGeofenceService.IzatGeofence gf = mGeofenceHandleDataMap.get(gfHandle);
            if (gf != null) {
                Object geofenceCtx = gfHandle.getContext();
                if ((geofenceCtx != null) && (geofenceCtx instanceof String)) {
                    editName.setText(geofenceCtx.toString());
                } else if ((geofenceCtx != null) && (geofenceCtx instanceof Bundle)) {
                    Bundle bundleObj = (Bundle) geofenceCtx;
                    bundleObj.setClassLoader(TestParcelable.class.getClassLoader());
                    TestParcelable testObj = bundleObj.getParcelable("bundle-obj");
                    if (testObj != null) {
                        if (VERBOSE) {
                            Log.v(TAG, "i = " + testObj.mitest + " s = " + testObj.mstest);
                        }
                    }
                    editName.setText("<<no name>>");
                }

                editLatitude.setText(Double.toString(gf.getLatitude()));
                editLongitude.setText(Double.toString(gf.getLongitude()));
                editRadius.setText(Double.toString(gf.getRadius()));
                editTransition.setText(Integer.toString(gf.getTransitionTypes().getValue()));

                IZatGeofenceService.IzatDwellNotify dwellNotify = gf.getDwellNotify();
                editDwellType.setText(Integer.toString(dwellNotify.getDwellType()));
                editDwellTime.setText(Integer.toString(dwellNotify.getDwellTime()));

                editConfidence.setText(Integer.toString(gf.getConfidence().getValue()));
                editResponsiveness.setText(Integer.toString(gf.getNotifyResponsiveness()));
            }
        }

        alertDialogBuilder.setPositiveButton("Save",
            new DialogInterface.OnClickListener() {
                public void onClick(DialogInterface dialog,int id) {
                    int transitionType = Integer.parseInt(editTransition.getText().toString());
                    int responsiveness = Integer.parseInt(editResponsiveness.getText().toString());

                    IZatGeofenceService.IzatGeofenceTransitionTypes gfTransitionType =
                    getTransitionEnumType(transitionType);

                    IZatGeofenceService.IzatGeofence gfObj = mGeofenceHandleDataMap.get(gfHandle);
                    if (gfObj != null) {
                        gfObj.setTransitionTypes(gfTransitionType);
                        gfObj.setNotifyResponsiveness(responsiveness);
                    }
                    gfHandle.update(gfTransitionType, responsiveness);
                }
            });

        alertDialogBuilder.setNegativeButton("Cancel",
            new DialogInterface.OnClickListener() {
                public void onClick(DialogInterface dialog,int id) {
                    dialog.cancel();
                }
            });

        // disable the non-editable fields and just display the current value of all fields
        editName.setFocusable(false);
        editLatitude.setFocusable(false);
        editLongitude.setFocusable(false);
        editRadius.setFocusable(false);
        editDwellType.setFocusable(false);
        editDwellTime.setFocusable(false);
        editConfidence.setFocusable(false);

        editName.setTextColor(Color.GRAY);
        editLatitude.setTextColor(Color.GRAY);
        editLongitude.setTextColor(Color.GRAY);
        editRadius.setTextColor(Color.GRAY);
        editDwellType.setTextColor(Color.GRAY);
        editDwellTime.setTextColor(Color.GRAY);
        editConfidence.setTextColor(Color.GRAY);

        // create alert dialog
        AlertDialog alertDialog = alertDialogBuilder.create();

        // show it
        alertDialog.show();
    }

    private void showAddDialog() {
        LayoutInflater inflater;
        View promptsView;

        // get the prompts view
        inflater = LayoutInflater.from(this);
        promptsView = inflater.inflate(R.layout.prompts, null);

        final EditText editName  = (EditText)(promptsView.findViewById(R.id.editAlertName));
        final EditText editLatitude = (EditText)(promptsView.findViewById(R.id.editTextLatitude));
        final EditText editLongitude = (EditText)(promptsView.findViewById(R.id.editTextLongitude));
        final EditText editRadius = (EditText)(promptsView.findViewById(R.id.editTextRadius));
        final EditText editTransition = (EditText)(promptsView.findViewById(R.id.editTextTransition));
        final EditText editDwellType = (EditText)(promptsView.findViewById(R.id.editTextDwellType));
        final EditText editDwellTime = (EditText)(promptsView.findViewById(R.id.editDwellTime));
        final EditText editConfidence = (EditText)(promptsView.findViewById(R.id.editConfidence));
        final EditText editResponsiveness = (EditText)(promptsView.findViewById(R.id.editResponsiveness));

        AlertDialog.Builder alertDialogBuilder =
            new AlertDialog.Builder(this);

        // set prompts.xml to alertdialog builder
        alertDialogBuilder.setView(promptsView);

        // set dialog message
        alertDialogBuilder.setCancelable(false);
        alertDialogBuilder.setPositiveButton("Save",
            new DialogInterface.OnClickListener() {
                public void onClick(DialogInterface dialog,int id) {
                    String geofenceName = editName.getText().toString();
                    double latitude = Double.parseDouble(editLatitude.getText().toString());
                    double longitude = Double.parseDouble(editLongitude.getText().toString());
                    double radius = Double.parseDouble(editRadius.getText().toString());
                    int transitionType = Integer.parseInt(editTransition.getText().toString());
                    int dwellType = Integer.parseInt(editDwellType.getText().toString());
                    int dwellTime = Integer.parseInt(editDwellTime.getText().toString());
                    int confidence = Integer.parseInt(editConfidence.getText().toString());
                    int responsiveness = Integer.parseInt(editResponsiveness.getText().toString());

                    IZatGeofenceService.IzatDwellNotify dwellNotify =
                    new IZatGeofenceService.IzatDwellNotify(dwellTime, dwellType);

                    IZatGeofenceService.IzatGeofence gf =
                    new IZatGeofenceService.IzatGeofence(latitude, longitude, radius);
                    gf.setTransitionTypes(getTransitionEnumType(transitionType));
                    gf.setConfidence(getConfidenceEnumType(confidence));
                    gf.setNotifyResponsiveness(responsiveness);
                    gf.setDwellNotify(dwellNotify);

                    Bundle bundleObj = null;
                    if (geofenceName.isEmpty()) {
                        TestParcelable testObj = new TestParcelable(50, "sample test string");
                        bundleObj = new Bundle(TestParcelable.class.getClassLoader());
                        bundleObj.putParcelable("bundle-obj", testObj);
                    }

                    IZatGeofenceService.IZatGeofenceHandle gfHandle =
                    mGfService.addGeofence(
                        geofenceName.isEmpty() ? bundleObj : geofenceName, gf);

                    if (gfHandle != null) {
                        mGeofenceHandleDataMap.put(gfHandle, gf);
                        Object gfCtx = gfHandle.getContext();
                        if ((gfCtx == null) || (!(gfCtx instanceof String))) {
                            gfCtx = "<<no name>>";
                        }
                        mGeofenceListNames.add(gfCtx.toString());
                        mGeofenceListadapter.notifyDataSetChanged();
                    }
                }
            });

        alertDialogBuilder.setNegativeButton("Cancel",
            new DialogInterface.OnClickListener() {
                public void onClick(DialogInterface dialog,int id) {
                    dialog.cancel();
                }
            });

        // create alert dialog
        AlertDialog alertDialog = alertDialogBuilder.create();

        // show it
        alertDialog.show();
    }

    private  IZatGeofenceService.IzatGeofenceTransitionTypes
            getTransitionEnumType(int transitionType) {
        IZatGeofenceService.IzatGeofenceTransitionTypes gfTransitionType;
        switch (transitionType) {
            case 0:
                gfTransitionType =
                        IZatGeofenceService.IzatGeofenceTransitionTypes.UNKNOWN;
            break;
            case 1:
                gfTransitionType =
                        IZatGeofenceService.IzatGeofenceTransitionTypes.ENTERED_ONLY;
            break;
            case 2:
                gfTransitionType =
                        IZatGeofenceService.IzatGeofenceTransitionTypes.EXITED_ONLY;
            break;
            case 3:
                gfTransitionType =
                        IZatGeofenceService.IzatGeofenceTransitionTypes.ENTERED_AND_EXITED;
            break;
            default:
                gfTransitionType =
                        IZatGeofenceService.IzatGeofenceTransitionTypes.UNKNOWN;
        }

        return gfTransitionType;
    }

    private IZatGeofenceService.IzatGeofenceConfidence
        getConfidenceEnumType(int confidence) {
        IZatGeofenceService.IzatGeofenceConfidence gfConfidence;
        switch (confidence) {
            case 1:
                gfConfidence = IZatGeofenceService.IzatGeofenceConfidence.LOW;
                break;
            case 2:
                gfConfidence = IZatGeofenceService.IzatGeofenceConfidence.MEDIUM;
                break;
            case 3:
                gfConfidence = IZatGeofenceService.IzatGeofenceConfidence.HIGH;
                break;
            default:
                gfConfidence = IZatGeofenceService.IzatGeofenceConfidence.LOW;
        }
        return gfConfidence;
   }
}
