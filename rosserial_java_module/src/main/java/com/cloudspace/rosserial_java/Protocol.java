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

package com.cloudspace.rosserial_java;


import org.jboss.netty.buffer.ChannelBuffer;
import org.jboss.netty.buffer.ChannelBuffers;
import org.ros.internal.message.Message;
import org.ros.internal.message.MessageBuffers;
import org.ros.message.MessageDeserializer;
import org.ros.message.MessageSerializer;
import org.ros.node.ConnectedNode;
import org.ros.node.parameter.ParameterTree;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;

import java.nio.ByteOrder;
import java.util.HashMap;
import java.util.Map;
import java.util.Timer;
import java.util.TimerTask;

import rosserial_msgs.Log;
import rosserial_msgs.RequestParam;
import rosserial_msgs.TopicInfo;
import std_msgs.Time;

/**
 * Protocol handler for rosserial.
 *
 * @author Adam Stambler
 */
public class Protocol {

    // SPECIAL IDS
    // All IDS greater than 100 are Publishers/Subscribers
    static final int TOPIC_PUBLISHERS = 0;
    static final int TOPIC_SUBSCRIBERS = 1;
    static final int TOPIC_TIME = 10;

    public static final byte[] NEGOTIATE_TOPICS_REQUEST = {
            (byte) 0xff, (byte) 0xfd, (byte) 0x00, (byte) 0x00, (byte) 0xff, (byte) 0x00, (byte) 0x00, (byte) 0xff};

    private static final String SERIAL_TAG = "ROS Serial";

    /**
     * Node hosting the subscribers and publishers.
     */
    private ConnectedNode node;

    /**
     * Map of IDs being sent down the channel to the topics they represent.
     */
    private Map<Integer, TopicInfo> id_to_topic = new HashMap<Integer, TopicInfo>();

    /**
     * Map of topic names to the IDs being sent down the channel for them.
     */
    private Map<String, Integer> topic_to_id = new HashMap<String, Integer>();

    /**
     * Topic ID to publisher.
     */
    private Map<Integer, Publisher> publishers = new HashMap<Integer, Publisher>();

    /**
     * Topic ID to subscriber.
     */
    private Map<Integer, Subscriber> subscribers = new HashMap<Integer, Subscriber>();

    /**
     * Topic ID to message deserializer for the associated topic message.
     */
    private Map<Integer, MessageDeserializer> msg_deserializers = new HashMap<Integer, MessageDeserializer>();

    /**
     * Listener for new publisher registrations.
     */
    private TopicRegistrationListener newPubListener;

    /**
     * Listener for new subscriber registrations.
     */
    private TopicRegistrationListener newSubListener;

    /**
     * Handles wire communication to the remote endpoint.
     */
    private PacketHandler packetHandler;

    public Protocol(ConnectedNode nh, PacketHandler handler) {
        this.node = nh;
        this.packetHandler = handler;
        this.paramT = nh.getParameterTree();
    }

    /**
     * Set a new topic registration listener for publications.
     *
     * @param listener
     */
    public void setOnNewPublication(TopicRegistrationListener listener) {
        newPubListener = listener;
    }

    /**
     * Set a new topic registration listener for subscriptions.
     *
     * @param listener
     */
    public void setOnNewSubcription(TopicRegistrationListener listener) {
        newSubListener = listener;
    }

    /**
     * Ask the remote endpoint for any topics it wants to publish or subscribe to.
     */
    public void negotiateTopics() {
        packetHandler.send(NEGOTIATE_TOPICS_REQUEST, TOPIC_PUBLISHERS);
    }

    /**
     * Construct a valid protocol message. This take the id and m, serializes
     * them and return the raw bytes to be sent
     * 
     * The MessageSerializer fills the 256 byte buffer with garbage 0x00 bytes. We trim all the
     *  0x00 bytes starting from the end, and if the result is an empty array, add one 0x00 to 
     *  prevent empty data  
     */
    public byte[] constructMessage(Message m) {
        ChannelBuffer buffer = MessageBuffers.dynamicBuffer();
        MessageSerializer serializer = node.getMessageSerializationFactory().newMessageSerializer(m.toRawMessage().getType());
        serializer.serialize(m, buffer);

        byte[] input = buffer.array();
        int i = input.length;
        while (i-- > 0 && input[i] == 0x00) {}

        byte[] output = new byte[i+1];
        System.arraycopy(input, 0, output, 0, i+1);
        if (output.length == 0) {
            output = new byte[] {0x00};
        }

        return output;
    }

    /**
     * ! Registers a topic being transmitted over the serial port /param topic-
     * The topic info msg describing the topic /param is_publisher - is the
     * device on the other end of the serial line publishing
     */
    private void addTopic(TopicInfo topic, boolean is_publisher, int id) {
        String name = topic.getTopicName();
        String type = topic.getMessageType();
        // check if its already registered
        if (id_to_topic.containsKey(id)) {
            if (id_to_topic.get(id).getTopicName().equals(name))
                return;
        }
        try {
            msg_deserializers.put(id,
                    node.getMessageSerializationFactory()
                            .newMessageDeserializer(type));
            topic_to_id.put(name, id);
            id_to_topic.put(id, topic);

            if (is_publisher) {
                Publisher pub = publishers.get(id);
                if (pub == null) {
                    pub = node.newPublisher(name, type);
                    publishers.put(id, pub);
                    android.util.Log.d("ADDING TOPIC", "Adding " + (is_publisher ? " publisher " : " subscriber ") + name + " of type " + type + " with id " + id);
                    if (newPubListener != null)
                        newPubListener.onNewTopic(topic);
                }
            } else {
                Subscriber sub = subscribers.get(id);
                if (sub == null) {
                    sub = node.newSubscriber(name, type);
                    sub.addMessageListener(new MessageListenerForwarding(id, this));
                    subscribers.put(id, sub);
                    android.util.Log.d("ADDING TOPIC", "Adding " + (is_publisher ? " publisher " : " subscriber ") + name + " of type " + type + " with id " + id);
                    if (newSubListener != null)
                        newSubListener.onNewTopic(topic);
                }

                node.getLog().info(
                        "Adding Subscriber " + name + " of type " + type);

            }
        } catch (Exception e) {
            node.getLog().error("Exception while adding topic", e);
        }
    }

    public TopicInfo[] getSubscriptions() {
        TopicInfo[] topics = new TopicInfo[subscribers.size()];

        int i = 0;
        for (Integer id : subscribers.keySet()) {
            topics[i++] = id_to_topic.get(id);
        }
        return topics;
    }

    public TopicInfo[] getPublications() {
        TopicInfo[] topics = new TopicInfo[publishers.size()];

        int i = 0;
        for (Integer id : publishers.keySet()) {
            topics[i++] = id_to_topic.get(id);
        }
        return topics;
    }

    /**
     * This timer handles monitoring handles monitoring the connection
     * to the device;
     */
    private Timer connection_timer = new Timer();
    static final int CONNECTION_TIMOUT_PERIOD = 20000;
    TimerTask timer_cb = new TimerTask() {

        @Override
        public void run() {
            // TODO Auto-generated method stub
            if (sync_requested) {
                sync_requested = false;

            } else {
                node.getLog().info("Connection to client lost. Topic negotiation requested");
                negotiateTopics();
            }
        }
    };

    private boolean sync_requested = false;

    public void start() {
        connection_timer.scheduleAtFixedRate(timer_cb, 0, CONNECTION_TIMOUT_PERIOD);
    }

    /**
     * Parse a packet from the remote endpoint.
     *
     * @param topic_id ID of the message topic.
     * @param buffer   The data for the message.
     * @return
     */
    public boolean parsePacket(int topic_id, byte[] buffer) {

        switch (topic_id) {
            case TopicInfo.ID_PUBLISHER:
            case TopicInfo.ID_SUBSCRIBER:
                android.util.Log.d("SUB BYTE " + topic_id, BinaryUtils.byteArrayToHexString(buffer));

                ChannelBuffer channelBuffer = ChannelBuffers.dynamicBuffer(ByteOrder.LITTLE_ENDIAN, 458881);
                channelBuffer.setBytes(0, buffer);
                channelBuffer.writerIndex(buffer.length + 1);
                MessageDeserializer<TopicInfo> deserializer = node.getMessageSerializationFactory().newMessageDeserializer(TopicInfo._TYPE);
                TopicInfo topic = deserializer.deserialize(channelBuffer);
                addTopic(topic, topic_id == TopicInfo.ID_PUBLISHER ? true : false, topic.getTopicId());
                break;

            case TopicInfo.ID_SERVICE_SERVER:
            case TopicInfo.ID_SERVICE_CLIENT:
                break;
            case TopicInfo.ID_PARAMETER_REQUEST:
                handleParameterRequest(buffer);
                break;
            case TopicInfo.ID_LOG:
                handleLogging(buffer);
                break;
            case TopicInfo.ID_TIME:
                sync_requested = true;
                org.ros.message.Time t = node.getCurrentTime();
                std_msgs.Time t_msg = node.getTopicMessageFactory().newFromType(Time._TYPE);
                t_msg.setData(t);
                packetHandler.send(constructMessage(t_msg), TOPIC_TIME);
                break;

            default:
                MessageDeserializer c = msg_deserializers.get(topic_id);
                if (c != null) {
                    ChannelBuffer messageBuffer = MessageBuffers.dynamicBuffer();
                    messageBuffer.setBytes(0, buffer);
                    messageBuffer.writerIndex(buffer.length + 1);
                    Message msg = (Message) c.deserialize(messageBuffer);
                    publishers.get(topic_id).publish(msg);
                } else {
                    node.getLog().info(
                            "Trying to publish to unregistered ID #" + topic_id);

                    // Try to negotiate topics then
                    negotiateTopics();
                }
                break;
        }

        return false;
    }

    /**
     * Handle Logging takes the log message from rosserial and rebroadcasts it
     * via rosout at the appropriate logging level
     *
     * @param buffer
     */
    private void handleLogging(byte[] buffer) {
        MessageDeserializer<Log> deserializer = node.getMessageSerializationFactory().newMessageDeserializer(Log._TYPE);
        ChannelBuffer messageBuffer = MessageBuffers.dynamicBuffer();
        messageBuffer.setBytes(0, buffer);
        messageBuffer.writerIndex(buffer.length + 1);
        Log msg = deserializer.deserialize(messageBuffer);
        String msgData = msg.getMsg();
        android.util.Log.d(SERIAL_TAG + " : " + msg.getLevel(), msgData);

        switch (msg.getLevel()) {
            case Log.ERROR:
                node.getLog().error(msgData);
                break;
            case Log.FATAL:
                node.getLog().fatal(msgData);
                break;
            case Log.INFO:
                node.getLog().info(msgData);
                break;
            case Log.ROSDEBUG:
                node.getLog().debug(msgData);
                break;
            case Log.WARN:
                node.getLog().warn(msgData);
                break;
        }
    }


    ParameterTree paramT;

    private void handleParameterRequest(byte[] buffer) {
//		RequestParam rp = new RequestParam();
//		RequestParam.Request req = rp.createRequest();
//		req.deserialize(msg_data);
        MessageDeserializer<RequestParam> deserializer = node.getMessageSerializationFactory().newMessageDeserializer(RequestParam._TYPE);
        ChannelBuffer messageBuffer = MessageBuffers.dynamicBuffer();
        messageBuffer.setBytes(0, buffer);
        messageBuffer.writerIndex(buffer.length + 1);
        RequestParam msg = deserializer.deserialize(messageBuffer);



//
//		RequestParam.Response resp = rp.createResponse();

    }

    /**
     * Forwards messages via a subscriber callback for a subscriber topic to the
     * packet handler which communicates with the remote endpoint.
     *
     * @author Adam Stambler
     */
    private static class MessageListenerForwarding<MessageType> implements
            org.ros.message.MessageListener<MessageType> {
        /**
         * The protocol handler handling the communication for the topic.
         */
        private Protocol protocol;

        /**
         * The topic ID for this listener.
         */
        private int id;

        public MessageListenerForwarding(int topic_id, Protocol p) {
            protocol = p;
            id = topic_id;
        }

        @Override
        public void onNewMessage(MessageType t) {
            android.util.Log.d("Forwarding Message", t.toString());
            protocol.packetHandler.send(protocol.constructMessage((Message) t), id);
        }
    }

    /**
     * Handles communication to the remote endpoint.
     *
     * @author Adam Stambler
     */
    public interface PacketHandler {
        /**
         * Send data to the remote endpoint.
         *
         * @param data The data to send.
         */
        void send(byte[] data, int topicId);
    }
}
