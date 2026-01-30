#!/usr/bin/env python3
import sys
# Force unbuffered output for piping
sys.stdout.reconfigure(line_buffering=True) if hasattr(sys.stdout, 'reconfigure') else None
"""
Serial Stream Helper for Mahony Filter Test

This script reads serial data from a connected board and outputs it to stdout,
making it easy to pipe to the visualization script.

Usage:
    python stream_serial.py <serial_port> [baud_rate]

Examples:
    # Windows
    python stream_serial.py COM5
    python stream_serial.py COM5 115200

    # Linux/Mac
    python stream_serial.py /dev/ttyACM0
    python stream_serial.py /dev/ttyUSB0 115200

    # Pipe to visualizer
    python stream_serial.py COM5 | python ../../test/test_mahony_filter/visualize_mahony.py --live

    # Save to file
    python stream_serial.py COM5 > mahony_data.csv
"""

import serial
import serial.tools.list_ports
import sys
import time
import argparse

def list_serial_ports():
    """List all available serial ports"""
    ports = serial.tools.list_ports.comports()
    return [(port.device, port.description) for port in ports]

def main():
    parser = argparse.ArgumentParser(
        description='Stream serial data to stdout for Mahony filter visualization',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__
    )
    parser.add_argument('port', nargs='?', help='Serial port (e.g., COM5 or /dev/ttyACM0)')
    parser.add_argument('baud', nargs='?', type=int, default=115200,
                       help='Baud rate (default: 115200)')
    parser.add_argument('--list', action='store_true',
                       help='List available serial ports')
    parser.add_argument('--timeout', type=float, default=1.0,
                       help='Serial read timeout in seconds (default: 1.0)')

    args = parser.parse_args()

    # List ports if requested
    if args.list:
        print("Available serial ports:", file=sys.stderr)
        ports = list_serial_ports()
        if not ports:
            print("  No serial ports found", file=sys.stderr)
        else:
            for device, description in ports:
                print(f"  {device}: {description}", file=sys.stderr)
        sys.exit(0)

    # Check if port was provided
    if not args.port:
        print("Error: Serial port required", file=sys.stderr)
        print("", file=sys.stderr)
        print("Available ports:", file=sys.stderr)
        ports = list_serial_ports()
        if not ports:
            print("  No serial ports found", file=sys.stderr)
        else:
            for device, description in ports:
                print(f"  {device}: {description}", file=sys.stderr)
        print("", file=sys.stderr)
        print("Usage: python stream_serial.py <port> [baud_rate]", file=sys.stderr)
        sys.exit(1)

    port = args.port
    baud = args.baud
    timeout = args.timeout

    # Try to open serial port
    try:
        print(f"# Connecting to {port} at {baud} baud...", file=sys.stderr)
        ser = serial.Serial(port, baud, timeout=timeout)
        time.sleep(2)  # Wait for board to reset after serial connection

        # Flush any initial garbage
        ser.reset_input_buffer()

        print(f"# Connected successfully", file=sys.stderr)
        print(f"# Reading data... (Ctrl+C to stop)", file=sys.stderr)
        print("#", file=sys.stderr)

        line_count = 0
        data_lines = 0

        while True:
            try:
                line = ser.readline()
                if line:
                    line_str = line.decode('utf-8', errors='ignore').strip()

                    # Output to stdout (for piping)
                    print(line_str)
                    sys.stdout.flush()  # Important for piping

                    # Count lines for status
                    line_count += 1
                    if not line_str.startswith('#') and ',' in line_str:
                        data_lines += 1

                    # Print status every 100 lines to stderr
                    if line_count % 100 == 0:
                        print(f"# [{line_count} lines, {data_lines} data points]",
                              file=sys.stderr)

            except UnicodeDecodeError:
                # Skip lines with encoding errors
                pass

    except serial.SerialException as e:
        print(f"# Error: Could not open serial port {port}", file=sys.stderr)
        print(f"# {e}", file=sys.stderr)
        print("#", file=sys.stderr)
        print("# Available ports:", file=sys.stderr)
        ports = list_serial_ports()
        for device, description in ports:
            print(f"#   {device}: {description}", file=sys.stderr)
        sys.exit(1)

    except KeyboardInterrupt:
        print("", file=sys.stderr)
        print(f"# Stopped by user", file=sys.stderr)
        print(f"# Total: {line_count} lines, {data_lines} data points", file=sys.stderr)

    except Exception as e:
        print(f"# Unexpected error: {e}", file=sys.stderr)
        sys.exit(1)

    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()
            print("# Serial port closed", file=sys.stderr)

if __name__ == '__main__':
    main()
