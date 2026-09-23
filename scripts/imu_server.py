import socket
import time
import pickle
from mini_bdx_runtime.imu import Imu
from mini_bdx_runtime.imu_i2c import DEFAULT_IMU_I2C_BUS
from threading import Thread
import time

import argparse


class IMUServer:
    def __init__(
        self, imu=None, pitch_bias=0, upside_down=False, i2c_bus=DEFAULT_IMU_I2C_BUS
    ):
        self.host = "0.0.0.0"
        self.port = 1234

        self.server_socket = socket.socket()
        self.server_socket.setsockopt(
            socket.SOL_SOCKET, socket.SO_REUSEADDR, 1
        )  # enable address reuse

        self.server_socket.bind((self.host, self.port))

        if imu is None:
            self.imu = Imu(
                50,
                user_pitch_bias=pitch_bias,
                upside_down=upside_down,
                i2c_bus=i2c_bus,
            )
        else:
            self.imu = imu
        self.stop = False

        Thread(target=self.run, daemon=True).start()

    def run(self):
        while not self.stop:
            self.server_socket.listen(1)
            conn, address = self.server_socket.accept()  # accept new connection
            print("Connection from: " + str(address))
            try:
                while True:
                    data = self.imu.get_data()
                    data = pickle.dumps(data)
                    conn.send(data)  # send data to the client
                    time.sleep(1 / 30)
            except:
                pass

        self.server_socket.close()
        print("thread closed")
        time.sleep(1)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--pitch_bias", type=float, default=0, help="deg")
    parser.add_argument("--i2c-bus", type=int, default=DEFAULT_IMU_I2C_BUS)
    parser.add_argument("--upside-down", action="store_true")
    args = parser.parse_args()
    imu_server = IMUServer(
        pitch_bias=args.pitch_bias,
        upside_down=args.upside_down,
        i2c_bus=args.i2c_bus,
    )
    try:
        while True:
            time.sleep(0.01)
    except KeyboardInterrupt:
        print("Closing server")
        imu_server.stop = True

    time.sleep(2)
