import usb.core
import usb.util
import subprocess
import time

def get_usb_devices():
    """
    Retrieve a list of all connected USB devices.

    Returns:
        list: List of connected USB devices.
    """
    devices = usb.core.find(find_all=True)
    device_list = []
    for device in devices:
        device_list.append(device)
    return device_list

def capture_usb_traffic(duration=10):
    """
    Capture USB traffic using the usbmon tool.

    Args:
        duration (int): Duration in seconds for which to capture USB traffic.

    Returns:
        float: Traffic in KB/s if successful, None otherwise.
    """
    try:
        cmd = ['/usr/bin/usbmon', '-t', str(duration), '-s', '512', '-i', '1']
        process = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        output, error = process.communicate()
        if error:
            print('Error while capturing traffic from usbmon:', error.decode())
            return None
        else:
            # Extract traffic data from usbmon output
            traffic = int(output.split(b'total_traffic: ')[-1].split()[0].decode())
            return traffic / 1024  # Convert to KB/s
    except Exception as e:
        print(f"Error capturing USB traffic: {e}")
        return None

def get_device_descriptor(device):
    """
    Get the descriptor of a USB device.

    Args:
        device: USB device object.

    Returns:
        The device descriptor if available, None otherwise.
    """
    try:
        # Extract the active configuration descriptor
        descriptor = device.get_active_configuration().extra
        return descriptor
    except Exception as e:
        print(f"Error getting device descriptor: {e}")
        return None

if __name__ == '__main__':
    # Retrieve all connected USB devices
    devices = get_usb_devices()
    
    for device in devices:
        try:
            # Get the device descriptor
            descriptor = get_device_descriptor(device)
            if descriptor:
                # Get the device class ID and convert it to a human-readable string
                class_id = device.bDeviceClass
                class_name = usb.util.get_string(device, class_id)
                print(f"Device Class: {class_name}")
            else:
                print("Device Class: Unknown")
            
            # Capture USB traffic for the device
            traffic = capture_usb_traffic()
            if traffic:
                print(f"Traffic: {traffic:.2f} KB/s")
            else:
                print("Traffic: Unable to capture")
        
        except usb.core.USBError as e:
            print("USBError:", e)
        except Exception as e:
            print("Error:", e)
        
        print("-" * 50)
        time.sleep(1)  # Wait for 1 second before checking the next device
