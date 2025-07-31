import os
import socket
import time
from threading import Thread

if os.environ.get("ENABLE_MAVLINK", "False").lower() == "true":
    from pymavlink import mavutil

class MavlinkClientWrapper:
    def __init__(self, app=None):
        self.app = app
        self.mavlink_threads = []
        self.enable_mavlink = os.environ.get("ENABLE_MAVLINK", "False").lower() == "true"
        self.mavlink_connections_number = int(os.environ.get("MAVLINK_CONNECTIONS_NUMBER", 10))
        self.out_addr = os.environ.get("OUT_ADDR", "localhost")

    def _mavlink_handler(self, in_port: int, out_port: int, out_addr: str):
        try:
            mavlink_connection = mavutil.mavlink_connection(f'udp:0.0.0.0:{in_port}')
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            print(f"MAVLink handler started for in_port {in_port} -> {out_addr}:{out_port}")

            while True:
                msg = mavlink_connection.recv_msg()
                if msg:
                    data = msg.get_msgbuf()
                    sock.sendto(data, (out_addr, out_port))
                time.sleep(1e-4)
        except Exception as e:
            print(f"Error in MAVLink handler for in_port {in_port}: {e}")

    def init_mavlink(self):
        if not self.enable_mavlink:
            print("MAVLink is disabled via environment variable ENABLE_MAVLINK.")
            return False

        print(f"Initializing MAVLink connections: {self.mavlink_connections_number} connections, output address: {self.out_addr}")

        connections = []
        start_in_port = 14551
        for i in range(self.mavlink_connections_number):
            in_port = start_in_port + i
            out_port = in_port + 20
            connections.append((in_port, out_port, self.out_addr))

        for conn_params in connections:
            in_port, out_port, out_addr = conn_params
            thread = Thread(
                name=f"mav_thread_{in_port}_{out_port}",
                target=self._mavlink_handler,
                args=(in_port, out_port, out_addr),
                daemon=True
            )
            self.mavlink_threads.append(thread)
            thread.start()
            print(f"MAVLink thread started: {thread.name}")
        
        if self.app:
            self.app.mavlink_client_wrapper = self
            
        return True

    def stop_mavlink(self):
        print("Stopping MAVLink client (daemonic threads will exit with main program).")
        pass

    def init_app(self, app):
        self.app = app
        if self.init_mavlink():
            print("Mavlink client initialized successfully with app.")
        else:
            print("Mavlink client initialization failed with app.")
        return self