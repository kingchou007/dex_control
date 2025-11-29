# filename: spacemouse_sender.py
import threading
import time
import zmq
import hid  # pip install hidapi

# --- CONFIG ---
# 0.0.0.0 means "Listen on all network interfaces"
ZMQ_PORT = 5555
# CHECK YOUR ID: Wireless=50735/50741, Wired=50770
PRODUCT_ID = 50741 
VENDOR_ID = 9583

# --- HELPERS ---
def to_int16(y1, y2):
    x = (y1) | (y2 << 8)
    if x >= 32768: x = -(65536 - x)
    return x

def scale(x, scale=350.0):
    return max(min(x / scale, 1.0), -1.0)

def convert(b1, b2):
    return scale(to_int16(b1, b2))

class SpaceMouseStreamer:
    def __init__(self):
        print(f"Opening SpaceMouse (PID: {PRODUCT_ID})...")
        try:
            self.device = hid.device()
            self.device.open(VENDOR_ID, PRODUCT_ID)
            self.device.set_nonblocking(1)
        except Exception as e:
            print(f"Error: {e}")
            exit(1)

        self.data = [0.0] * 6 # [x, y, z, r, p, y]
        self.running = True
        
        # Start HID Thread
        self.thread = threading.Thread(target=self._run_hid, daemon=True)
        self.thread.start()

    def _run_hid(self):
        while self.running:
            try:
                d = self.device.read(13)
                if d:
                    if d[0] == 1: # Translation
                        self.data[0] = convert(d[3], d[4]) # X
                        self.data[1] = convert(d[1], d[2]) # Y
                        self.data[2] = convert(d[5], d[6]) * -1.0 # Z
                        
                        # Handle models that send rotation in packet 1
                        if len(d) > 12:
                            self.data[3] = convert(d[7], d[8])
                            self.data[4] = convert(d[9], d[10])
                            self.data[5] = convert(d[11], d[12])

                    elif d[0] == 2: # Rotation (packet 2)
                        self.data[3] = convert(d[1], d[2]) # Roll
                        self.data[4] = convert(d[3], d[4]) # Pitch
                        self.data[5] = convert(d[5], d[6]) # Yaw
            except:
                pass

    def get_data(self):
        return self.data

def main():
    # Setup ZMQ Publisher
    context = zmq.Context()
    socket = context.socket(zmq.PUB)
    socket.bind(f"tcp://0.0.0.0:{ZMQ_PORT}")
    
    print(f"Streaming SpaceMouse on Port {ZMQ_PORT}...")
    sm = SpaceMouseStreamer()
    
    try:
        while True:
            # 1. Get Data
            raw = sm.get_data()
            
            # 2. Send as JSON
            msg = {"axes": raw}
            socket.send_json(msg)
            
            # 3. Rate Limit (100Hz is plenty)
            time.sleep(0.01)
            
    except KeyboardInterrupt:
        print("\nStopping...")

if __name__ == "__main__":
    main()