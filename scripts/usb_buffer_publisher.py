#!/usr/bin/env python
import rospy
from std_msgs.msg import UInt8MultiArray
import usb.core
import usb.util
import pyudev


class UsbDevice():
    def __init__(self, vendor_id, product_id, serial_number):
        self.vendor_id = vendor_id
        self.product_id = product_id
        self.serial_number = serial_number
        self.dev = None
        self.intf = None
        self.buffer = None
        self.pub = None

    def add_device(device):
        # Check if device is a USB device
        if device.subsystem != 'usb':
            return

        # Create new UsbDevice instance
        new_device = UsbDevice(device.get('ID_VENDOR_ID'), device.get('ID_PRODUCT_ID'), device.get('ID_SERIAL_SHORT'))

        # Add device to devices list
        devices.append(new_device)

        # Initialize publisher for device
        new_device.pub = rospy.Publisher('usb_data_' + new_device.serial_number, UInt8MultiArray, queue_size=10)

        # Connect to USB device
        try:
            new_device.dev = usb.core.find(idVendor=new_device.vendor_id, idProduct=new_device.product_id, serial_number=new_device.serial_number)
            if new_device.dev is None:
                rospy.logerr("USB device not found: VendorID={0:#x}, ProductID={1:#x}, SerialNumber={2}".format(new_device.vendor_id, new_device.product_id, new_device.serial_number))
                return

            # Set configuration
            new_device.dev.set_configuration()

            # Find USB interface
            new_device.intf = usb.util.find_descriptor(new_device.dev, bInterfaceNumber=0)

            # Claim interface
            usb.util.claim_interface(new_device.dev, new_device.intf)

            # Initialize buffer
            new_device.buffer = bytearray(64)
        except usb.core.USBError as e:
            rospy.logerr("USB error occurred for device {0}: {1}".format(new_device.serial_number, str(e)))

def usb_publisher():
    # Initialize ROS node
    rospy.init_node('usb_publisher_node', anonymous=True)

    # Define USB devices and their buffers
    devices = [
        UsbDevice(0x1234, 0x5678, "ABCD"),
        UsbDevice(0x1234, 0x5679, "EFGH")
    ]

    # Initialize publishers for each USB device
    for device in devices:
        device.pub = rospy.Publisher('usb_data_' + device.serial_number, UInt8MultiArray, queue_size=10)

    # Connect to USB devices
    for device in devices:
        dev = usb.core.find(idVendor=device.vendor_id, idProduct=device.product_id, serial_number=device.serial_number)
        if dev is None:
            rospy.logerr("USB device not found: VendorID={0:#x}, ProductID={1:#x}, SerialNumber={2}".format(device.vendor_id, device.product_id, device.serial_number))
            return

        device.dev = dev

        # Set configuration
        dev.set_configuration()

        # Find USB interface
        intf = usb.util.find_descriptor(dev, bInterfaceNumber=0)

        # Claim interface
        usb.util.claim_interface(dev, intf)
        device.intf = intf

        # Initialize buffer
        device.buffer = bytearray(64)

    # Read data from USB devices and publish it
    while not rospy.is_shutdown():
        for device in devices:
            try:
                # Read data from USB device
                size = device.dev.read(0x81, device.buffer, timeout=1000)
                usb_msg = UInt8MultiArray(data=list(device.buffer[:size]))

                # Publish data as ROS message
                device.pub.publish(usb_msg)
            except usb.core.USBError as e:
                rospy.logerr("USB error occurred for device {0}: {1}".format(device.serial_number, str(e)))
                break

    # Release interfaces and close USB devices
    for device in devices:
        if device.intf is not None:
            usb.util.release_interface(device.dev, device.intf)
        if device.dev is not None:
            device.dev.reset()

if __name__ == '__main__':
    try:
        usb_publisher()
    except rospy.ROSInterruptException:
        pass
