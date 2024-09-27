import pusher
import serial

pusher_client = pusher.Pusher(
    # app_id='your-app-id',
    # key='your-app-key',
    # secret='your-app-secret',
    # cluster='your-cluster',
    ssl=True
)

def read_serial():
    # ser = serial.Serial('/dev/ttyUSB0', 9600)
    while True:
        data = ser.readline()
        if data:
            decoded_data = data.decode('utf-8')
            print(decoded_data)
            pusher_client.trigger('my-channel', 'my-event', {'message': decoded_data})

if __name__ == "__main__":
    read_serial()
