/*
*    Copyright (c) 2014 Qualcomm Technologies, Inc. All Rights Reserved.
*    Qualcomm Technologies Proprietary and Confidential.
*
*/
package com.qualcomm.qti.vehicle.explorer;

import android.app.Activity;
import android.content.Intent;
import android.os.Bundle;
import android.os.Handler;
import android.os.Message;

public class SplashActivity extends Activity {

   @Override
   public void onCreate(Bundle savedInstanceState) {
       super.onCreate(savedInstanceState);
       setContentView(R.layout.splash);
       mHandler = new SplashHandler();
       mHandler.sendMessageDelayed(new Message(), 1000);
   }
   private class SplashHandler extends Handler {
       public void handleMessage(Message msg) {
           Intent intent = new Intent(SplashActivity.this, VehicleExplorer.class);
           SplashActivity.this.finish();
           startActivity(intent);
       }
   }
   private SplashHandler mHandler = null;
}
