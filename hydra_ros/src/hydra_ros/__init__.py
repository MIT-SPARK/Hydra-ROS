"""ROS Python Package for Hydra-ROS."""

from typing import Optional
import rospy
import spark_dsg as dsg
import hydra_msgs.msg
import std_msgs.msg


class DsgPublisher:
    """Class for publishing a scene graph from python."""

    def __init__(self, topic, publish_mesh: bool = True):
        """Construct a sender."""
        self._publish_mesh = publish_mesh
        self._pub = rospy.Publisher(topic, hydra_msgs.msg.DsgUpdate, queue_size=1)

    def publish(self, G, stamp: Optional[rospy.Time] = None, frame_id: str = "odom"):
        """Send a graph."""
        header = std_msgs.msg.Header()
        header.stamp = stamp if stamp is not None else rospy.Time.now()
        header.frame_id = frame_id
        self.publish_with_header(G, header)

    def publish_with_header(self, G, header):
        """Send a graph."""
        if self._pub.get_num_connections() > 0:
            msg = hydra_msgs.msg.DsgUpdate()
            msg.header = header
            msg.layer_contents = G.to_binary(self._publish_mesh)
            msg.full_update = True
            self._pub.publish(msg)


class DsgSubscriber:
    """Class for receiving a scene graph in python."""

    def __init__(self, topic, callback):
        """Construct a DSG Receiver."""
        self._callback = callback
        self._graph_set = False
        self._graph = None

        self._sub = rospy.Subscriber(
            topic, hydra_msgs.msg.DsgUpdate, self._handle_update
        )

    def _handle_update(self, msg):
        if not msg.full_update:
            raise NotImplementedError("Partial updates not implemented yet")

        size_bytes = len(msg.layer_contents)
        rospy.logdebug(f"Received dsg update message of {size_bytes} bytes")

        if not self._graph_set:
            self._graph = dsg.DynamicSceneGraph.from_binary(msg.layer_contents)
            self._graph_set = True
        else:
            self._graph.update_from_binary(msg.layer_contents)

        self._callback(msg.header, self._graph)
