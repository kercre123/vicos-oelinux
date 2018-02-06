/*
 * Copyright (c) 2015 Qualcomm Technologies, Inc.
 * All Rights Reserved.
 * Qualcomm Technologies Proprietary and Confidential.
 */
package com.qualcomm.qti.vehicle.explorer.tests;

import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.net.ServerSocket;
import java.net.Socket;
import java.nio.charset.Charset;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;

import android.util.Log;

public class SocketCommunicator {
    private static final String TAG = "VehicleServiceSocketCommunicator";
    private final boolean mDebug;
    private final int mPort;
    private final int mBufferLength;
    private ServerSocket mServerSocket;
    private Socket mClientSocket;
    private OutputStream mOut;
    private InputStream mIn;
    private boolean mConnected = false; // Indicates if we are connected

    public static void printDebugMsgs(Exception e){
            Log.i(TAG, "Hit exception " + e);
            Log.i(TAG, "Hit exception " + e.getMessage());
            Log.e(TAG, "STACKTRACE");
            Log.e(TAG, Log.getStackTraceString(e));
    }

    public SocketCommunicator(int port, int bufferLength, boolean debug) {
        mDebug = debug;
        mPort = port;
        mBufferLength = bufferLength;
        connectSocket();
    }

    private boolean connectSocket() {
        try {
            Log.i(TAG, "Opening socket on port " + mPort);
            mServerSocket = new ServerSocket(mPort);
            mServerSocket.setReuseAddress(true);
            // block until someone connects to socket
            mClientSocket = mServerSocket.accept();
            if (mDebug) Log.i(TAG, "got connection on port " + mPort);
            mOut = mClientSocket.getOutputStream();
            mIn = mClientSocket.getInputStream();
            mConnected = true;
            Log.i(TAG, "SocketCommunicator connected");
        } catch (Exception e) {
            Log.e(TAG, "Connection failed! " + e);
            disconnect();
            return false;
        }
        return true;
    }

    private void closeSocket() {
        mConnected = false;
        try {
            Log.i(TAG, "Closing socket");
            if (mOut != null) mOut.close();
            if (mIn != null) mIn.close();
            if (mServerSocket != null) mServerSocket.close();
            if (mClientSocket != null) mClientSocket.close();
        } catch (Exception e) {
            Log.i(TAG, "Exception occurred while closing socket");
            printDebugMsgs(e);
        }
        mOut = null;
        mIn = null;
        mClientSocket = null;
        mServerSocket = null;
        Log.i(TAG, "SocketCommunicator disconnected");
    }

    public byte[] read() {
        byte[] buffer = null;
        if (mConnected && mIn != null) {
            try {
                if (mDebug) Log.i(TAG, "readerLoop is waiting for message");
                buffer = readBytes(4); // reading int length
                int messageSize = b2i(buffer);
                if (mDebug) Log.i(TAG, "reading message size: " + messageSize);
                if (messageSize < 1) {
                    throw new IOException("read that messageSize is " + messageSize);
                }
                buffer = readBytes(messageSize);
            } catch (Exception e) {
                Log.e(TAG, "readerLoop: " + e);
                e.printStackTrace();
                closeSocket();
            }
        }
        return buffer;
    }

    /*
     * This makes sure to read specified number of bytes or throw exception.
     * No more and no less. It's either read what asked or throw exception.
     */
    private byte[] readBytes(int length) throws IOException {
        byte[] buffer = new byte[length];
        int totalBytesTransferred = 0, readBytes;
        do {
            readBytes = mIn.read(buffer, 0, length);
            totalBytesTransferred += readBytes;
        } while (readBytes > 0 && totalBytesTransferred < length);
        if (readBytes <= 0) {
            Log.i(TAG, "bytes read: " + readBytes);
            throw new IOException("Last read returned EOF:0");
        }
        return buffer;
    }

    public void connect() {
        if (mConnected) {
            return;
        }
        connectSocket();
    }

    public void disconnect() {
        if (!mConnected) {
            return;
        }
        closeSocket();
    }

    public boolean isConnected(){
        return mConnected;
    }

    public boolean sendMessage(byte[] buffer) {
        if (mDebug) Log.i(TAG, "SC: sendMessage " + buffer.length);
        if (!mConnected || mOut == null) {
            return false;
        }
        try {
            int length = buffer.length;
            mOut.write(i2b(length));
            mOut.write(buffer);
        } catch (IOException e) {
            Log.i(TAG, "SocketCommunicator failed to send " + e);
            closeSocket();
            return false;
        }
        if (mDebug) Log.i(TAG, "SC: sendMessage success");
        return true;
    }

    public static byte[] i2b(int i) {
        byte[] result = new byte[4];
        result[0] = (byte)(i & 0xFF);
        result[1] = (byte)(i>>8 & 0xFF);
        result[2] = (byte)(i>>16 & 0xFF);
        result[3] = (byte)(i>>24 & 0xFF);
        return result;
    }

    public static int b2i(byte[] b) {
        int i = b[0] & 0xFF |
                b[1] << 8 & 0xFF00 |
                b[2] << 16 & 0xFF0000 |
                b[3] << 24 & 0xFF000000;
        return i;
    }
}