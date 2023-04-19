#!/usr/bin/env python

import rospy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
import psutil
import pyudev

def monitor_usb_buffer():
    context = pyudev.Context()
    usb_devices = [device for device in context.list_devices(subsystem='usb')]
    pub = rospy.Publisher('/diagnostics_usb', DiagnosticArray, queue_size=10)

    while not rospy.is_shutdown():
        partitions = psutil.disk_partitions()
        msg = DiagnosticArray()
        msg.header.stamp = rospy.get_rostime()

        for usb_device in usb_devices:
            usb_status = DiagnosticStatus()
            usb_status.name = f"{usb_device.get('ID_VENDOR')} {usb_device.get('ID_MODEL')}"

            partition_found = False
            for partition in partitions:
                if partition.mountpoint.startswith(usb_device.get('UDISKS2_MP_MOUNT_POINTS', '')):
                    partition_found = True
                    usage = psutil.disk_usage(partition.mountpoint)
                    buffer_usage = usage.percent
                    buffer_bar = "[" + "=" * int(buffer_usage // 5) + " " * int((100 - buffer_usage) // 5) + "]"
                    usb_status.message = f"Buffer usage: {buffer_usage}% {buffer_bar}"
                    usb_status.level = DiagnosticStatus.OK if buffer_usage < 80 else DiagnosticStatus.WARN
                    usb_status.values.append(KeyValue("Device path", partition.device))
                    usb_status.values.append(KeyValue("Mount point", partition.mountpoint))
                    break

            if not partition_found:
                usb_status.message = "No partition found"
                usb_status.level = DiagnosticStatus.ERROR

            msg.status.append(usb_status)

        pub.publish(msg)

        rospy.sleep(1)

if __name__ == '__main__':
    rospy.init_node('usb_buffer_monitor')
    try:
        monitor_usb_buffer()
    except rospy.ROSInterruptException:
        pass
