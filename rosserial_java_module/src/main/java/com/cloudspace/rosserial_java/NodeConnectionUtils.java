package com.cloudspace.rosserial_java;

import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;

public class NodeConnectionUtils extends AbstractNodeMain {
    String name;
    OnNodeConnectedListener listener;
    
    public static interface OnNodeConnectedListener {
        void onNodeConnected(ConnectedNode node);
        
    }
    public NodeConnectionUtils(String name, OnNodeConnectedListener listener) {
        this.name = name;
        this.listener = listener;
    }

    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of(name);
    }

    @Override
    public void onStart(ConnectedNode connectedNode) {
        listener.onNodeConnected(connectedNode);
    }
}
