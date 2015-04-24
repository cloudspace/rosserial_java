package com.cloudspace.rosserial_java;

import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;

public class NodeConnectionUtils extends AbstractNodeMain {
    OnNodeConnectedListener listener;
    
    public interface OnNodeConnectedListener {
        void onNodeConnected(ConnectedNode node);
        
    }
    public NodeConnectionUtils(OnNodeConnectedListener listener) {
        this.listener = listener;
    }

    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.newAnonymous();
    }

    @Override
    public void onStart(ConnectedNode connectedNode) {
        listener.onNodeConnected(connectedNode);
    }
}
