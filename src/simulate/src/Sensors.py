#!/usr/bin/env python

import rospy
import traceback
import time
from std_msgs.msg import Int32


def sensors_reader():
    # Launch a node called "sensors_node"
    rospy.init_node('sensors_node', anonymous=False)

    # Create the publishers. Name each topic "sensors_##", with message type "Int32"
    # (because that's what is received over the serial port)
    # Note the queue_size=1 statement: don't let it develop a backlog!
    publishers = {
        "A0": rospy.Publisher('/sensors_A0', Int32, queue_size=1),
        "A1": rospy.Publisher('/sensors_A1', Int32, queue_size=1),
        "A2": rospy.Publisher('/sensors_A2', Int32, queue_size=1),
        "A3": rospy.Publisher('/sensors_A3', Int32, queue_size=1),
        "A4": rospy.Publisher('/sensors_A4', Int32, queue_size=1),
        "A5": rospy.Publisher('/sensors_A5', Int32, queue_size=1),
        "E0": rospy.Publisher('/sensors_E0', Int32, queue_size=1),
        "E1": rospy.Publisher('/sensors_E1', Int32, queue_size=1),
        "U0": rospy.Publisher('/sensors_U0', Int32, queue_size=1),
        "U1": rospy.Publisher('/sensors_U1', Int32, queue_size=1),
        "U2": rospy.Publisher('/sensors_U2', Int32, queue_size=1),
    }

    # Declare the message that will go on the topic.
    # Here we use one of the message name types we Imported, and add parentheses to call it as a function.
    # We put data in it using the .data field of the message.
    msgs = {
        "A0": Int32(),
        "A1": Int32(),
        "A2": Int32(),
        "A3": Int32(),
        "A4": Int32(),
        "A5": Int32(),
        "E0": Int32(),
        "E1": Int32(),
        "U0": Int32(),
        "U1": Int32(),
        "U2": Int32(),
    }

    # MAIN LOOP to keep loading the message with new data.
    # NOTE that at the moment the data are coming from a separate thread, but this will be replaced with the serial port line reader in the future.
    data = {
        "A0": 1,
        "A1": 0,
        "A2": 0,
        "A3": 0,
        "A4": 0,
        "A5": 0,
        "E0": 0,
        "E1": 0,
        "U0": 0,
        "U1": 0,
        "U2": 0,
    }

    while not rospy.is_shutdown():
        time.sleep(0.5)

        # Have the encoders going forward
        data["E0"] += 50
        data["E1"] += 50

        try:
            for key in data.keys():
                msgs[key] = data[key]
                publishers[key].publish(msgs[key])

        except Exception:
            traceback.print_exc()
            pass


if __name__ == '__main__':
    try:
        sensors_reader()
    except rospy.ROSInterruptException:
        pass
