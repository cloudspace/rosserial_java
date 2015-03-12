// Software License Agreement (BSD License)
//
// Copyright (c) 2011, Willow Garage, Inc.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above
//    copyright notice, this list of conditions and the following
//    disclaimer in the documentation and/or other materials provided
//    with the distribution.
//  * Neither the name of Willow Garage, Inc. nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
// Author: Adam Stambler  <adasta@gmail.com>
// http://wiki.ros.org/rosserial/Overview/Protocol
package com.cloudspace.rosserial_java;

import android.util.Log;

import org.ros.node.ConnectedNode;

import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.util.Timer;

import rosserial_msgs.TopicInfo;

/**
 * The host computer endpoint for a rosserial connection.
 *
 * @author Adam Stambler
 */
public class ROSSerial implements Runnable {
    /**
     * Flags for marking beginning of packet transmission
     */
    public static final byte[] FLAGS = {(byte) 0xff, (byte) 0xfd};

    /**
     * Maximum size for the incomming message data in bytes
     * Same as Message out buffer size in rosserial_arduino
     */
    private static final int MAX_MSG_DATA_SIZE = 2048;

    /**
     * Output stream for the serial line used for communication.
     */
    private OutputStream ostream;

    /**
     * Input stream for the serial line used for communication.
     */
    private InputStream istream;

    /**
     * The node which is hosting the publishers and subscribers.
     */
    private ConnectedNode node;

    /**
     * Protocol handler being used for this connection.
     */
    private Protocol protocol;

    private final Object lock = new Object();

    /**
     * Set a new topic registration listener for publications.
     *
     * @param listener
     */
    public void setOnNewPublication(TopicRegistrationListener listener) {
        protocol.setOnNewPublication(listener);
    }

    /**
     * Set a new topic registration listener for subscriptions.
     *
     * @param listener
     */
    public void setOnNewSubcription(TopicRegistrationListener listener) {
        protocol.setOnNewSubcription(listener);
    }


    public TopicInfo[] getSubscriptions() {
        return protocol.getSubscriptions();
    }

    public TopicInfo[] getPublications() {
        return protocol.getPublications();
    }

    /**
     * True if this endpoint is running, false otherwise.
     */
    private boolean running = false;

    // parsing state machine variables/enumes
    private enum PACKET_STATE {
        FLAGA, FLAGB, MESSAGE_LENGTH, LENGTH_CHECKSUM, TOPIC_ID, DATA, MSG_CHECKSUM
    }

    private PACKET_STATE packet_state;
    private byte[] topicIdBytes = new byte[2];
    private byte[] messageLengthBytes = new byte[2];
    private int data_len = 0;
    private int byte_index = 0;
    byte[] buffer = new byte[256];

    /**
     * Packet handler for writing to the other endpoint.
     */
    private Protocol.PacketHandler sendHandler = new Protocol.PacketHandler() {

        @Override
        public void send(byte[] data, int topicId) {
            byte[] packet = generatePacket(data, topicId);
            Log.d("SENDING PACKET @ " + packet.length, BinaryUtils.byteArrayToHexString(packet));
            try {
                ostream.write(packet);
            } catch (Exception e) {
                e.printStackTrace();
            }
        }
    };

    private byte[] generatePacket(byte[] data, int topicId) {
        int length = data.length;            
        byte tHigh = (byte) ((topicId & 0xFF00) >> 8);
        byte tLow = (byte) (topicId & 0xFF);
        int dataValues = 0;
             
        dataValues += tLow;
        dataValues += tHigh;
        for (int i = 0; i < data.length; i++) {
            dataValues += 0xff & data[i];
        }

        byte lHigh = (byte) ((length & 0xFF00) >> 8);
        byte lLow = (byte) (length & 0xFF);

        byte lengthChk = (byte) (255 - length % 256);
        byte dataChk = (byte) (255 - dataValues % 256);
               
        byte[] almost = new byte[]{FLAGS[0], FLAGS[1], lLow, lHigh, lengthChk, tLow, tHigh};

        byte[] result = new byte[almost.length + data.length + 1];
        System.arraycopy(almost, 0, result, 0, almost.length);
        System.arraycopy(data, 0, result, almost.length, data.length);
        result[result.length - 1] = dataChk;

        return result;

    }

    public ROSSerial(ConnectedNode nh, InputStream input, OutputStream output) {
        ostream = output;
        istream = input;
        node = nh;
        protocol = new Protocol(node, sendHandler);

    }

    private byte[] data;

    /**
     * Shut this endpoint down.
     */
    public void shutdown() {
        running = false;
    }

    /**
     * This timer watches when a packet starts.  If the packet
     * does not complete itself within 30 milliseconds
     * the message is thrown away.
     */
    Timer packet_timeout_timer;

    /**
     * Start running the endpoint.
     */
    public void run() {
        synchronized (lock) {
            protocol.start();

            resetPacket();

            running = true;

            // TODO
            // there should be a node.isOk() or something
            // similar so that it stops when ros is gone
            // but node.isOk() does not work, its never true...
            while (running) {
                try {
                    int bytes = istream.read(buffer);
                    if (bytes > 8 && packet_state == PACKET_STATE.FLAGA) {
                        data = new byte[(buffer[3] << 8) | (buffer[2])];
                        int i = 0;
                        for (i = 0; i < data.length + 8; i++) {
                            handleByte(buffer[i]);
                        }
                    } else {
                        resetPacket();
                    }
                } catch (IOException e) {
                    node.getLog().error("Unable to read input stream", e);
                    System.out.println("Unable to read input stream");

                    if (e.toString().equals("java.io.IOException: No such device")) {
                        node.getLog()
                                .error("Total IO Failure.  Now exiting ROSSerial iothread.");
                        break;
                    }
                    resetPacket();
                } catch (Exception e) {
                    node.getLog().error("Unable to read input stream", e);
                }
                try {
                    //Sleep prevents continuous polling of istream.
                    //continuous polling kills an inputstream on android
                    Thread.sleep(10);
                } catch (InterruptedException e) {
                    break;
                }
            }
            node.getLog().info("Finished ROSSerial IO Thread");
        }
    }

    /*
     * ! reset parsing statemachine
     */
    private void resetPacket() {
        byte_index = 0;
        data_len = 0;
        messageLengthBytes = new byte[2];
        topicIdBytes = new byte[2];
        packet_state = PACKET_STATE.FLAGA;
    }

    /*
     * ! handle byte takes an input byte and feeds it into the parsing
     * statemachine /param b input byte /return true or falls depending on if
     * the byte was successfully parsed
     */
    private boolean handleByte(byte b) {
//        Log.d("HANDLE BYTE", byte_index + " : " + BinaryUtils.byteToHexString(b) + " : " + packet_state);
        switch (packet_state) {
            case FLAGA:
                if (b == (byte) 0xff) {
                    packet_state = PACKET_STATE.FLAGB;
                }
                break;
            case FLAGB:
                if (b == (byte) 0xfd) {
                    packet_state = PACKET_STATE.MESSAGE_LENGTH;
                } else {
                    resetPacket();
                    return false;
                }
                break;
            case MESSAGE_LENGTH:
                messageLengthBytes[byte_index] = b;
                byte_index++;
                if (byte_index == 2) {
                    byte_index = 0;
                    packet_state = PACKET_STATE.LENGTH_CHECKSUM;
                }
                break;

            case LENGTH_CHECKSUM:
                data_len = (messageLengthBytes[1] << 8) | (messageLengthBytes[0]);
                int dataChk = 255 - data_len % 256;
                if ((byte) dataChk == b) {
                    Log.d("Checksum succeeded!", BinaryUtils.byteToHexString(b));
                    packet_state = PACKET_STATE.TOPIC_ID;
                    byte_index = 0;
                } else {
                    Log.d("Checksum failed!", BinaryUtils.byteToHexString(b) + " : " + String.valueOf(dataChk));
                    resetPacket();
                }
                break;
            case TOPIC_ID:
                topicIdBytes[byte_index] = b;
                byte_index++;
                if (byte_index == 2) {
                    byte_index = 0;
                    packet_state = PACKET_STATE.DATA;
                }
                break;
            case DATA:
                data[byte_index] = b;
                byte_index++;
                if (byte_index == data_len) {
                    packet_state = PACKET_STATE.MSG_CHECKSUM;
                }
                break;
            case MSG_CHECKSUM:
                int chk = 0;
                for (int i = 0; i < 2; i++)
                    chk += (int) (0xff & topicIdBytes[i]);
                for (int i = 0; i < data_len; i++) {
                    chk += (int) (0xff & data[i]);
                }

                if ((byte) chk == b) {
                    resetPacket();
                    Log.d("Checksum failed!", BinaryUtils.byteToHexString(b) + " : " + String.valueOf(chk));
                    resetPacket();
                    return false;
                } else {
                    Log.d("Checksum succeeded!", BinaryUtils.byteToHexString(b));
                }

                int topic_id = (topicIdBytes[1] << 8) | (topicIdBytes[0]);
                protocol.parsePacket(topic_id, data);
                resetPacket();
                break;
        }
        return true;
    }
}