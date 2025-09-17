
import zmq
import sys
import threading
import json
from MAVProxy.modules.lib import mp_module
import socket

# --- ZMQ Configuration ---
PUB_PORT = 5556
REP_PORT = 5557

class StdoutCapture:
    """
    A file-like object to capture stdout, write it to the original stdout,
    and publish it to a ZMQ PUB socket.
    """
    def __init__(self, original_stdout, pub_socket):
        self.original_stdout = original_stdout
        self.pub_socket = pub_socket

    def write(self, text):
        # Write to the original console so the user can still see it
        self.original_stdout.write(text)
        self.original_stdout.flush()
        # Publish the text to any ZMQ subscribers
        if text.strip():  # Avoid sending empty lines
            self.pub_socket.send_string(text)

    def flush(self):
        self.original_stdout.flush()

class APIBridgeModule(mp_module.MPModule):
    """
    The main MAVProxy module for the ZMQ API bridge.
    """
    def __init__(self, mpstate):
        super(APIBridgeModule, self).__init__(mpstate, "api_bridge", "ZMQ API Bridge")


        self.system_status = {"links": {}}
        self.max_link_num = 0

        try:
            # --- ZMQ Context Setup ---
            self.context = zmq.Context()
            
            # --- Publisher for console output ---
            self.pub_socket = self.context.socket(zmq.PUB)
            self.pub_socket.bind(f"tcp://*:{PUB_PORT}")
            
            # --- Capture and redirect stdout ---
            self.original_stdout = sys.stdout
            self.stdout_capture = StdoutCapture(self.original_stdout, self.pub_socket)
            sys.stdout = self.stdout_capture
            
            self.log("ZMQ API Bridge: Publishing console output on tcp://*:%u" % PUB_PORT)

            # --- Replier for commands ---
            self.rep_socket = self.context.socket(zmq.REP)
            self.rep_socket.bind(f"tcp://*:{REP_PORT}")
            self.log("ZMQ API Bridge: Listening for commands on tcp://*:%u" % REP_PORT)
            
            # --- Start REP loop in a background thread ---
            self.rep_thread = threading.Thread(target=self.rep_loop, daemon=True)
            self.rep_thread.start()
            self.log("ZMQ API Bridge: Module loaded successfully.")

        except Exception as e:
            # If something fails, restore stdout and log the error
            if hasattr(self, 'original_stdout') and self.original_stdout:
                sys.stdout = self.original_stdout
            print(f"ZMQ API Bridge: Failed to initialize - {e}")


    def rep_loop(self):
        """
        The main loop for the REP socket, handling incoming requests.
        """
        while True:
            try:
                message_str = self.rep_socket.recv_string()
                request = json.loads(message_str)
                
                action = request.get("action")
                reply = {"status": "ERROR", "message": "Unknown action"}

                if action == "get_status":
                    output = self.handle_get_status()
                    reply = {"status": "success", "output": output}
                
                elif action == "run_command":
                    command = request.get("command")
                    if command:
                        output = self.handle_run_command(command)
                        reply = {"status": "success", "output": output}
                    else:
                        reply = {"status": "ERROR", "message": "No command provided"}
                
                self.rep_socket.send_json(reply)

            except zmq.error.ContextTerminated:
                # Context was terminated, so exit the thread gracefully
                break
            except Exception as e:
                self.log(f"ZMQ REP Error: {e}")
                error_reply = {"status": "ERROR", "message": str(e)}
                # We must always send a reply in the REP/REQ pattern
                self.rep_socket.send_json(error_reply)
    
    def mavlink_packet(self, msg):
        '''handle an incoming mavlink packet'''
        type = msg.get_type()
        if type in ['HEARTBEAT', 'HIGH_LATENCY2']:
            self.handle_heartbeat(msg)

    def handle_heartbeat(self, msg):
            sysid = msg.get_srcSystem()
            compid = msg.get_srcComponent()
            master = self.master

            fmode = master.flightmode
            
            if self.settings.vehicle_name:
                fmode = self.settings.vehicle_name + ':' + fmode
            
            self.system_status["armed"] = self.master.motors_armed()
            self.system_status["flight_mode"] = fmode

            
            
            if self.max_link_num != len(self.mpstate.mav_master):
                self.max_link_num = len(self.mpstate.mav_master)

            
            for m in self.mpstate.mav_master:
                
                if self.mpstate.settings.checkdelay:
                    highest_msec_key = (sysid, compid)
                    linkdelay = (self.mpstate.status.highest_msec.get(highest_msec_key, 0) - m.highest_msec.get(highest_msec_key,0))*1.0e-3
                else:
                    linkdelay = 0
                linkline = "Link %s " % (self.link_label(m))
                fg = 'dark green'

                if m.linkerror:
                    linkline += "down"
                    fg = 'red'
                    signal_strength = {
                        "id": m.linknum,
                        "label": self.link_label(m),
                        "status": "down",
                        "link_quality_percent": 0,
                        "packets_sent": m.mav_count,
                        "packets_received": m.mav_loss,
                        "color": fg,
                        "delay": linkdelay
                    }

                else:
                    packets_rcvd_percentage = 100
                    if (m.mav_count+m.mav_loss) != 0: #avoid divide-by-zero
                        packets_rcvd_percentage = (100.0 * m.mav_count) / (m.mav_count + m.mav_loss)

                    linkbits = ["%u pkts" % m.mav_count,
                                "%u lost" % m.mav_loss,
                                "%.2fs delay" % linkdelay,
                    ]
                    try:
                        if m.mav.signing.sig_count:
                            # other end is sending us signed packets
                            if not m.mav.signing.secret_key:
                                # we've received signed packets but
                                # can't verify them
                                fg = 'orange'
                                linkbits.append("!KEY")
                            elif not m.mav.signing.sign_outgoing:
                                # we've received signed packets but aren't
                                # signing outselves; this can lead to hairloss
                                fg = 'orange'
                                linkbits.append("!SIGNING")
                            if m.mav.signing.badsig_count:
                                fg = 'orange'
                                linkbits.append("%u badsigs" % m.mav.signing.badsig_count)
                    except AttributeError as e:
                        # mav.signing.sig_count probably doesn't exist
                        pass

                    linkline += "OK {rcv_pct:.1f}% ({bits})".format(
                        rcv_pct=packets_rcvd_percentage,
                        bits=", ".join(linkbits))

                    if linkdelay > 1 and fg == 'dark green':
                        fg = 'orange'

                    signal_strength = {
                        "id": m.linknum,
                        "label": self.link_label(m),
                        "status": "OK",
                        "link_quality_percent": packets_rcvd_percentage,
                        "packets_sent": m.mav_count,
                        "packets_received": m.mav_loss,
                        "color": fg,
                        "delay": linkdelay
                    }
                
                self.system_status["links"][m.linknum] = signal_strength


    def handle_get_status(self):
        """
        Handles the 'get_status' action.
        """
        outputs = []
        for i in range(len(self.mpstate.mav_outputs)):
            conn = self.mpstate.mav_outputs[i]
            outputs.append(conn.address)

        output_string = ", ".join(outputs)

        status = self.system_status.copy()
        # Get LAN IP address (not loopback) only once and cache it
        if not hasattr(self, "_server_ip_mp"):
            hostname = socket.gethostname()
            try:
                # This connects to a dummy address to get the LAN IP
                s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                s.connect(("8.8.8.8", 80))
                self._server_ip_mp = s.getsockname()[0]
                s.close()
            except Exception:
                self._server_ip_mp = socket.gethostbyname(hostname)
        server_ip = self._server_ip_mp

        status["connection_info"] = {
            "server_ip": f"{server_ip}",
            "mavlink_outputs": output_string
        }

        return status


    def handle_run_command(self, command):
        """
        Handles the 'run_command' action.
        """
        self.log(f"Running command: '{command}'")
        # Use MAVProxy's own command handling function
        self.mpstate.functions.process_stdin(command)
        output = {"message": f"Command '{command}' executed"}
        self.say(f"Executed command: {command}")
        return output

    def unload(self):
        """
        Called when the module is unloaded.
        """
        # Restore stdout and close sockets to clean up
        if sys.stdout is self.stdout_capture:
            sys.stdout = self.original_stdout
        
        self.log("Unloading module, closing ZMQ sockets.")
        # Close sockets and terminate context
        self.pub_socket.close()
        self.rep_socket.close()
        self.context.term()
        self.log("Module unloaded.")

    def log(self, message):
        """
        Helper for logging to the original console.
        """
        print(f"ZMQ API Bridge: {message}", file=self.original_stdout)


def init(mpstate):
    '''initialise module'''
    return APIBridgeModule(mpstate)
