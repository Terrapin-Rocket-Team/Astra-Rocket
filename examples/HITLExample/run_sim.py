import argparse
import time
import sys
import datetime
import csv
import matplotlib.pyplot as plt
import astra_link
import astra_sim

def parse_telem_header(header_line):
    """Parses 'TELEM/Time,Alt,...' into a map and a list"""
    if not header_line.startswith("TELEM/"): return None, None
    raw = header_line[6:].strip()
    parts = [p.strip() for p in raw.split(',')]
    col_map = {name: i for i, name in enumerate(parts)}
    return col_map, parts

def main():
    parser = argparse.ArgumentParser(description="Astra Rocket Handshake Sim")
    parser.add_argument('--mode', choices=['hitl', 'sitl'], required=True, help="Connection mode")
    parser.add_argument('--source', choices=['physics', 'csv', 'net'], default='physics', help="Data source")
    parser.add_argument('--port', help="Serial port (HITL)")
    parser.add_argument('--baud', type=int, default=115200, help="Baud rate")
    parser.add_argument('--host', default='localhost', help="TCP Host (SITL)")
    parser.add_argument('--tcp-port', type=int, default=5555, help="TCP Port (SITL)")
    parser.add_argument('--file', help="CSV file path")
    parser.add_argument('--udp-port', type=int, default=9000, help="External UDP port")

    args = parser.parse_args()

    # --- 1. Setup Link ---
    try:
        if args.mode == 'hitl':
            if not args.port: raise ValueError("--port required for HITL")
            link = astra_link.SerialLink(args.port, args.baud)
        else:
            link = astra_link.TCPLink(args.host, args.tcp_port)
    except Exception as e:
        print(f"Connection Failed: {e}")
        sys.exit(1)

    # --- 2. Setup Source ---
    try:
        if args.source == 'csv':
            if not args.file: raise ValueError("--file required for CSV mode")
            sim = astra_sim.CSVSim(args.file)
        elif args.source == 'net':
            sim = astra_sim.NetworkStreamSim(args.udp_port)
        else:
            sim = astra_sim.PhysicsSim()
    except Exception as e:
        print(f"Sim Setup Failed: {e}")
        link.close()
        sys.exit(1)

    # --- 3. Handshake Phase ---
    print("\n[Init] Waiting for Flight Computer Header...")
    print("       (Please reset/boot the Flight Computer now)")
    
    fc_col_map = None
    fc_header_names = []
    
    # Wait indefinitely for the header
    while True:
        line = link.read_line()
        if line and line.startswith("TELEM/") and ("State" in line or "Time" in line):
            fc_col_map, fc_header_names = parse_telem_header(line)
            print(f"[Init] Header Received! Found {len(fc_header_names)} columns.")
            print(f"[Init] Columns: {fc_header_names}")
            break
        time.sleep(0.01)

    print("[Init] Handshake Complete. Starting Simulation in 1s...")
    time.sleep(1.0)

    # --- 4. Main Loop ---
    print("\nStarting LOCK-STEP Simulation")
    print(f"{'SimT':<6} | {'SimAlt':<8} | {'FC State':<12} | {'FC Alt':<8} | {'Events'}")
    print("-" * 65)

    history = {'time': [], 'sim_alt': [], 'fc_alt': [], 'fc_stage': [], 'fc_values': []}
    last_stage = "BOOT"
    pkt_count = 0
    
    try:
        while True:
            # A. Get Next Packet
            if sim.is_finished():
                print("\n[Sim] CSV Finished. Exiting Loop.")
                break
            
            packet = sim.get_next_packet()
            pkt_count += 1
            msg = packet.to_hitl_string().encode()

            # B. Send & Wait (Retry Logic)
            attempts = 0
            max_retries = 3
            fc_response_line = None
            
            while attempts < max_retries:
                try:
                    link.send(msg)
                except ConnectionError:
                    print("\n[Link] Connection Died.")
                    break

                # Wait for Response (1.0s timeout)
                start_wait = time.time()
                got_response = False
                
                while (time.time() - start_wait) < 1.0:
                    line = link.read_line()
                    if line and line.startswith("TELEM/"):
                        fc_response_line = line
                        got_response = True
                        break
                
                if got_response:
                    break
                else:
                    attempts += 1
            
            if attempts >= max_retries:
                if pkt_count % 50 == 0:
                     print(f"\r[FC] Timeout - Packet {pkt_count} skipped.\033[K", end='')
                continue # Skip this step

            # C. Parse Response
            fc_alt_val = 0.0
            fc_stage_val = last_stage 
            current_values = []
            
            if fc_response_line:
                raw_content = fc_response_line[6:].strip()
                current_values = [x.strip() for x in raw_content.split(',')]
                
                if fc_col_map:
                    def get_fc(keys_list, default):
                        for k in keys_list:
                            if k in fc_col_map and fc_col_map[k] < len(current_values):
                                return current_values[fc_col_map[k]]
                        return default

                    fc_alt_str = get_fc(["State - PZ (m)", "Alt", "State - Alt (m)"], "0")
                    temp_stage = get_fc(["State - Flight Stage", "Stage"], last_stage)
                    if "-" in temp_stage: temp_stage = temp_stage.split('-')[-1].strip()
                    fc_stage_val = temp_stage
                    try: fc_alt_val = float(fc_alt_str)
                    except: pass

            # D. Store & Display
            history['time'].append(packet.timestamp)
            history['sim_alt'].append(packet.alt)
            history['fc_alt'].append(fc_alt_val)
            history['fc_stage'].append(fc_stage_val)
            history['fc_values'].append(current_values)

            is_event = (fc_stage_val != last_stage)
            status_str = f"{packet.timestamp:6.2f} | {packet.alt:8.1f} | {fc_stage_val:12} | {fc_alt_val:8.1f}"
            
            if is_event:
                print(f"\r{status_str} | STAGE: {fc_stage_val}\033[K")
            elif pkt_count % 50 == 0: 
                sys.stdout.write(f"\r{status_str} | \033[K")
                sys.stdout.flush()

            last_stage = fc_stage_val

    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        link.close()
        print("\nSimulation Closed.")
        
        # --- Save CSV Log ---
        timestamp_str = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        log_name = f"sim_log_{timestamp_str}.csv"
        print(f"Saving log to {log_name}...")
        
        try:
            with open(log_name, 'w', newline='') as f:
                writer = csv.writer(f)
                
                # Header
                csv_header = ['Sim_Time', 'Sim_Alt']
                if fc_header_names: csv_header.extend(fc_header_names)
                else: csv_header.append("FC_Raw_Data")
                writer.writerow(csv_header)

                # Rows
                for i in range(len(history['time'])):
                    row = [history['time'][i], history['sim_alt'][i]]
                    fc_row_data = history['fc_values'][i]
                    
                    if fc_header_names:
                        if len(fc_row_data) >= len(fc_header_names):
                            row.extend(fc_row_data[:len(fc_header_names)])
                        else:
                            row.extend(fc_row_data + [''] * (len(fc_header_names) - len(fc_row_data)))
                    else:
                        row.extend(fc_row_data)
                    writer.writerow(row)
        except Exception as e:
            print(f"Error saving log: {e}")

        # --- Plot ---
        print("Plotting results...")
        try:
            plt.figure(figsize=(10, 6))
            plt.plot(history['time'], history['sim_alt'], label='Sim Truth (m)', color='blue', alpha=0.6)
            clean_fc = [x if x != 0 else None for x in history['fc_alt']]
            plt.plot(history['time'], clean_fc, label='FC KF Estimate (m)', color='orange', linestyle='--')
            
            for i in range(1, len(history['fc_stage'])):
                if history['fc_stage'][i] != history['fc_stage'][i-1]:
                    plt.axvline(x=history['time'][i], color='gray', linestyle=':', alpha=0.5)
            
            plt.title("Simulation vs Flight Computer Estimate")
            plt.xlabel("Time (s)")
            plt.ylabel("Altitude (m)")
            plt.grid(True)
            plt.legend()
            plt.show()
        except: pass

if __name__ == "__main__":
    main()