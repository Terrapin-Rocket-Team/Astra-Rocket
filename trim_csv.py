#!/usr/bin/env python3
"""
Script to trim flight data CSV, removing pre-launch and post-landing data.
Keeps data from 5 seconds before launch to 5 seconds after landing.
"""

import csv
import sys
import argparse


def find_launch_and_landing(rows, alt_column, time_column, threshold=5.0, buffer_sec=5.0):
    """
    Find launch and landing indices based on altitude threshold.

    Args:
        rows: List of CSV row dicts
        alt_column: Name of altitude column
        time_column: Name of time column
        threshold: Altitude threshold in meters to detect launch/landing
        buffer_sec: Seconds of buffer to add before launch and after landing

    Returns:
        (start_idx, end_idx) tuple
    """
    # Find launch: first time altitude goes above threshold
    launch_idx = None
    for i, row in enumerate(rows):
        try:
            alt = float(row[alt_column])
            if alt > threshold:
                launch_idx = i
                break
        except (ValueError, KeyError):
            continue

    if launch_idx is None:
        print("Error: Could not find launch point (altitude never exceeded threshold)")
        return None, None

    # Find landing: last time altitude is above threshold
    landing_idx = None
    for i in range(len(rows) - 1, -1, -1):
        try:
            alt = float(rows[i][alt_column])
            if alt > threshold:
                landing_idx = i
                break
        except (ValueError, KeyError):
            continue

    if landing_idx is None:
        print("Error: Could not find landing point")
        return None, None

    # Apply time buffer
    launch_time = float(rows[launch_idx][time_column])
    landing_time = float(rows[landing_idx][time_column])

    target_start_time = launch_time - buffer_sec
    target_end_time = landing_time + buffer_sec

    # Find closest indices to target times
    start_idx = launch_idx
    for i in range(launch_idx, -1, -1):
        if float(rows[i][time_column]) <= target_start_time:
            start_idx = i
            break

    end_idx = landing_idx
    for i in range(landing_idx, len(rows)):
        if float(rows[i][time_column]) >= target_end_time:
            end_idx = i
            break

    print(f"Launch detected at row {launch_idx}, time {launch_time:.2f}s")
    print(f"Landing detected at row {landing_idx}, time {landing_time:.2f}s")
    print(f"Keeping data from row {start_idx} (t={float(rows[start_idx][time_column]):.2f}s) "
          f"to row {end_idx} (t={float(rows[end_idx][time_column]):.2f}s)")

    return start_idx, end_idx


def trim_csv(input_file, output_file, alt_column='MS5611 - Alt ASL (m)',
             time_column='State - Time (s)', threshold=5.0, buffer_sec=5.0):
    """
    Trim CSV file to remove pre-launch and post-landing data.
    Normalizes timestamps to start at 0.

    Args:
        input_file: Path to input CSV file
        output_file: Path to output CSV file
        alt_column: Name of altitude column
        time_column: Name of time column
        threshold: Altitude threshold in meters
        buffer_sec: Seconds of buffer before launch and after landing
    """
    # Read CSV
    print(f"Reading {input_file}...")
    with open(input_file, 'r', encoding='utf-8-sig', errors='ignore') as f:
        reader = csv.DictReader(f)
        rows = list(reader)
        fieldnames = reader.fieldnames

    print(f"Loaded {len(rows)} rows")

    # Find launch and landing
    start_idx, end_idx = find_launch_and_landing(rows, alt_column, time_column,
                                                   threshold, buffer_sec)

    if start_idx is None or end_idx is None:
        print("Error: Could not determine trim points")
        sys.exit(1)

    # Trim data
    trimmed_rows = rows[start_idx:end_idx+1]
    print(f"Trimmed to {len(trimmed_rows)} rows ({len(rows) - len(trimmed_rows)} removed)")

    # Normalize timestamps to start at 0
    # PadDelaySim will add 5s pad delay before CSV playback, so:
    # - Simulation time 0-5s: Pad delay (repeats first CSV packet)
    # - Simulation time 5s+: CSV playback (CSV time 0 becomes sim time 5, etc.)
    if time_column in trimmed_rows[0]:
        time_offset = float(trimmed_rows[0][time_column])
        print(f"Normalizing timestamps (offset: {time_offset:.2f}s)")

        for row in trimmed_rows:
            try:
                original_time = float(row[time_column])
                normalized_time = original_time - time_offset
                row[time_column] = f"{normalized_time:.3f}"
            except (ValueError, KeyError):
                pass

        last_time = float(trimmed_rows[-1][time_column])
        print(f"CSV time range: 0.000s to {last_time:.3f}s")
        print(f"Simulation time range (with 5s pad delay): 0.000s to {last_time + 5.0:.3f}s")

    # Write output
    print(f"Writing to {output_file}...")
    with open(output_file, 'w', newline='', encoding='utf-8') as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(trimmed_rows)

    print("Done!")


def main():
    parser = argparse.ArgumentParser(
        description='Trim flight data CSV to remove pre-launch and post-landing data'
    )
    parser.add_argument('input', help='Input CSV file')
    parser.add_argument('output', nargs='?', help='Output CSV file (default: input_trimmed.csv)')
    parser.add_argument('--threshold', type=float, default=5.0,
                       help='Altitude threshold in meters (default: 5.0)')
    parser.add_argument('--buffer', type=float, default=5.0,
                       help='Time buffer in seconds before launch and after landing (default: 5.0)')
    parser.add_argument('--alt-column', default='MS5611 - Alt ASL (m)',
                       help='Name of altitude column')
    parser.add_argument('--time-column', default='State - Time (s)',
                       help='Name of time column')

    args = parser.parse_args()

    # Default output filename
    if args.output is None:
        if args.input.endswith('.csv'):
            args.output = args.input[:-4] + '_trimmed.csv'
        else:
            args.output = args.input + '_trimmed.csv'

    trim_csv(args.input, args.output, args.alt_column, args.time_column,
             args.threshold, args.buffer)


if __name__ == '__main__':
    main()
