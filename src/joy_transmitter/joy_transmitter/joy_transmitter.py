import serial
import struct
from time import sleep
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy


class JoystickSubscriber(Node):
    def __init__(self):
        super().__init__('joystick_subscriber')
        self.subscription = self.create_subscription(
            Joy,
            'joy',  # The topic name; make sure it matches the topic you want to subscribe to
            self.joystick_callback,
            10  # QoS history depth
        )
        self.subscription  # prevent unused variable warning
        self.joystick_axes = []  # Initialize an empty list to store joystick axes

    def joystick_callback(self, msg):
        # Update the axes data when a new message is received
        self.joystick_axes = msg.axes


def send_zero_values(ser):
    """Send zero values for velocity and rotation before shutdown."""
    try:
        zero_buff = struct.pack('=BBf', 36, 36, 0.0)
        ser.write(zero_buff)
        sleep(0.01)
    except serial.SerialException as e:
        print(f"Failed to send zero values: {e}")


def main(args=None):
    rclpy.init(args=args)
    joystick_subscriber = JoystickSubscriber()

    # Try to open the serial port
    try:
        ser = serial.Serial(
            # port='COM3',  # Windows
            port='/dev/ttyUSB0', # Linux, run "dmesg | grep -i usb" to find the port
            baudrate=115200,
            timeout=0.1
        )
    except serial.SerialException as e:
        print(f"Error opening serial port: {e}")
        exit(1)

    try:
        # Main loop to process joystick data and serial communication
        while rclpy.ok():
            # Spin to process callbacks
            rclpy.spin_once(joystick_subscriber, timeout_sec=0.1)

            # Access and print the joystick data
            if joystick_subscriber.joystick_axes:
                testTransmit = joystick_subscriber.joystick_axes[1]
                print(f'{testTransmit}')
            else:
                testTransmit = 0.0
                print("No joystick data available.")

            try:
                # Pack data into a byte buffer ('=BBf' = '=(Byte)(Byte)(float)')
                buff = struct.pack('=BBf', 36, 36, testTransmit)

                # Send data over serial
                ser.write(buff)

                # Read the incoming data
                input = ser.read(40)

                # Check if sufficient data was read
                if len(input) < 40:
                    print("Incomplete data received")
                    continue

                ser.reset_input_buffer()

                # Find the position of the header
                headerPos = input.find(b'\x24\x24')

                # Ensure the header was found
                if headerPos == -1:
                    print("Header not found in received data")
                    continue

                # Extract and unpack the data after the header (header = two byte, message = 4 byte)
                inData = input[(headerPos + 2):(headerPos + 6)]

                # Ensure that the expected number of bytes is available for unpacking
                if len(inData) != 4:
                    print("Incomplete float data received")
                    continue

                # ('=f' = '=(float)')
                receivedValues = struct.unpack('=f', inData)

                testReceive = receivedValues

                # Print the received values
                print(f"Receive value : {testReceive}")

            except struct.error as e:
                print(f"Error unpacking data: {e}")
            except serial.SerialException as e:
                print(f"Serial communication error: {e}")

            # Sleep for 0.1 second before the next loop iteration
            sleep(0.1)

    except KeyboardInterrupt:
        print("Shutdown requested by user...")

    finally:
        # Send zero values before shutdown
        ser.flush()
        for i in range(100):
            send_zero_values(ser)
        sleep(1)

        # Close the serial port
        if ser.is_open:
            ser.close()
            print("Serial port closed.")

        joystick_subscriber.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


