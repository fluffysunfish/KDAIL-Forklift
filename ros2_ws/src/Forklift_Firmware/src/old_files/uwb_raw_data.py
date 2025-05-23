import serial
import re
import time
import math
from collections import deque
import statistics

class UWBSLAMReader:
    def __init__(self, port, baudrate=115200):
        self.ser = serial.Serial(port, baudrate, timeout=1.0, rtscts=True)

        # Anchor positions (x, y) in cm
        self.anchor1 = (0, 0)    # d1
        self.anchor2 = (0, 170)      # d2
        self.anchor3 = (200, 170)    # d3

        # Filter parameters optimized for SLAM bot
        self.filter_window = 4     # Smaller window for better responsiveness
        self.max_change_threshold = 50  # Reasonable for slow-moving bot (10cm-500cm range)
        self.min_valid_readings = 2     # Faster response

        # Filter buffers
        self.d1_buffer = deque(maxlen=self.filter_window)
        self.d2_buffer = deque(maxlen=self.filter_window)
        self.d3_buffer = deque(maxlen=self.filter_window)

        # Last valid readings
        self.last_d1 = None
        self.last_d2 = None
        self.last_d3 = None

        # Position history for better selection
        self.last_position = None

        # Regex patterns
        self.d1_pattern = re.compile(r'mac_address=0x0001.*?distance\[cm\]=(\d+)')
        self.d2_pattern = re.compile(r'mac_address=0x0002.*?distance\[cm\]=(\d+)')
        self.d3_pattern = re.compile(r'mac_address=0x0003.*?distance\[cm\]=(\d+)')

    def is_valid_reading(self, new_val, last_val):
        """Check if reading is within acceptable change threshold"""
        if last_val is None or new_val is None:
            return True
        return abs(new_val - last_val) <= self.max_change_threshold

    def apply_filter(self, buffer):
        """Apply median filter to buffer"""
        if len(buffer) < self.min_valid_readings:
            return None
        return statistics.median(buffer)

    def calculate_position_from_pair(self, p1, d1, p2, d2):
        """
        Calculate position from two anchor points and distances
        Returns list of (x, y) positions or empty list if no valid solution
        """
        x1, y1 = p1
        x2, y2 = p2

        # Distance between anchors
        D = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

        # Check if circles intersect
        if d1 + d2 < D or abs(d1 - d2) > D or D == 0:
            return []

        try:
            # Calculate intersection points
            a = (d1**2 - d2**2 + D**2) / (2 * D)
            h_squared = d1**2 - a**2

            # Check if h_squared is negative (no real solution)
            if h_squared < 0:
                return []

            h = math.sqrt(h_squared)

            # Point on line between anchors
            px = x1 + a * (x2 - x1) / D
            py = y1 + a * (y2 - y1) / D

            # Two possible intersection points
            x_pos1 = px + h * (y2 - y1) / D
            y_pos1 = py - h * (x2 - x1) / D

            x_pos2 = px - h * (y2 - y1) / D
            y_pos2 = py + h * (x2 - x1) / D

            positions = []

            # Add both positions if they're different
            if abs(h) > 0.1:  # Not tangent case
                positions.append((x_pos1, y_pos1))
                positions.append((x_pos2, y_pos2))
            else:  # Tangent case - only one solution
                positions.append((x_pos1, y_pos1))

            return positions

        except (ValueError, ZeroDivisionError):
            return []

    def select_best_position(self, positions):
        """
        Select the best position from multiple candidates
        """
        if not positions:
            return None

        if len(positions) == 1:
            return positions[0]

        # Filter positions within reasonable bounds (with some tolerance)
        bounds_filtered = []
        for x, y in positions:
            if -50 <= x <= 220 and -50 <= y <= 250:  # Allow some tolerance
                bounds_filtered.append((x, y))

        if not bounds_filtered:
            bounds_filtered = positions  # Use all if none within bounds

        # If we have a previous position, choose the closest one
        if self.last_position and len(bounds_filtered) > 1:
            last_x, last_y = self.last_position
            min_dist = float('inf')
            best_pos = bounds_filtered[0]

            for x, y in bounds_filtered:
                dist = math.sqrt((x - last_x)**2 + (y - last_y)**2)
                if dist < min_dist:
                    min_dist = dist
                    best_pos = (x, y)

            return best_pos

        # Otherwise, prefer position closer to center of area
        center = (85, 100)
        min_dist = float('inf')
        best_pos = bounds_filtered[0]

        for x, y in bounds_filtered:
            dist = math.sqrt((x - center[0])**2 + (y - center[1])**2)
            if dist < min_dist:
                min_dist = dist
                best_pos = (x, y)

        return best_pos

    def calculate_position(self, d1, d2, d3):
        """
        Calculate tag position using trilateration
        Returns (x, y) or None if insufficient data
        Works with any two or more distances
        """
        all_positions = []

        # Calculate from d1-d2 pair
        if d1 is not None and d2 is not None:
            positions = self.calculate_position_from_pair(self.anchor1, d1, self.anchor2, d2)
            all_positions.extend([(pos, 'd1-d2') for pos in positions])

        # Calculate from d1-d3 pair
        if d1 is not None and d3 is not None:
            positions = self.calculate_position_from_pair(self.anchor1, d1, self.anchor3, d3)
            all_positions.extend([(pos, 'd1-d3') for pos in positions])

        # Calculate from d2-d3 pair
        if d2 is not None and d3 is not None:
            positions = self.calculate_position_from_pair(self.anchor2, d2, self.anchor3, d3)
            all_positions.extend([(pos, 'd2-d3') for pos in positions])

        if not all_positions:
            return None

        # Extract just the positions for processing
        candidate_positions = [pos for pos, source in all_positions]

        # If we have positions from multiple anchor pairs, try to find consensus
        if len(set(source for pos, source in all_positions)) > 1:
            # Group positions by proximity (within 20cm considered same)
            position_clusters = []
            for pos in candidate_positions:
                added_to_cluster = False
                for cluster in position_clusters:
                    # Check if position is close to any position in this cluster
                    for cluster_pos in cluster:
                        dist = math.sqrt((pos[0] - cluster_pos[0])**2 + (pos[1] - cluster_pos[1])**2)
                        if dist < 20:  # 20cm threshold
                            cluster.append(pos)
                            added_to_cluster = True
                            break
                    if added_to_cluster:
                        break

                if not added_to_cluster:
                    position_clusters.append([pos])

            # Find the largest cluster (most consensus)
            if position_clusters:
                largest_cluster = max(position_clusters, key=len)
                if len(largest_cluster) > 1:
                    # Average positions in the largest cluster
                    avg_x = sum(pos[0] for pos in largest_cluster) / len(largest_cluster)
                    avg_y = sum(pos[1] for pos in largest_cluster) / len(largest_cluster)
                    result = (avg_x, avg_y)
                else:
                    result = self.select_best_position(candidate_positions)
            else:
                result = self.select_best_position(candidate_positions)
        else:
            # Only one anchor pair available
            result = self.select_best_position(candidate_positions)

        # Update last position if we got a valid result
        if result:
            self.last_position = result

        return result

    def parse_distances(self, data):
        """Extract d1, d2, d3 from raw data"""
        d1 = d2 = d3 = None

        d1_match = self.d1_pattern.search(data)
        d2_match = self.d2_pattern.search(data)
        d3_match = self.d3_pattern.search(data)

        if d1_match:
            d1_raw = int(d1_match.group(1))
            if self.is_valid_reading(d1_raw, self.last_d1):
                d1 = d1_raw
                self.last_d1 = d1_raw

        if d2_match:
            d2_raw = int(d2_match.group(1))
            if self.is_valid_reading(d2_raw, self.last_d2):
                d2 = d2_raw
                self.last_d2 = d2_raw

        if d3_match:
            d3_raw = int(d3_match.group(1))
            if self.is_valid_reading(d3_raw, self.last_d3):
                d3 = d3_raw
                self.last_d3 = d3_raw

        return d1, d2, d3

    def get_filtered_distances(self):
        """Get current filtered distance values"""
        d1_filtered = self.apply_filter(self.d1_buffer)
        d2_filtered = self.apply_filter(self.d2_buffer)
        d3_filtered = self.apply_filter(self.d3_buffer)

        return d1_filtered, d2_filtered, d3_filtered

    def count_available_distances(self, d1, d2, d3):
        """Count how many distances are available"""
        count = 0
        sources = []
        if d1 is not None:
            count += 1
            sources.append('d1')
        if d2 is not None:
            count += 1
            sources.append('d2')
        if d3 is not None:
            count += 1
            sources.append('d3')
        return count, sources

    def read_positions(self):
        """Main reading loop with position calculation"""
        print("UWB SLAM Position Reader")
        print("Anchors: A1(0,600), A2(0,0), A3(400,0)")
        print("Now supports positioning with any 2+ anchors")
        print("Press Ctrl+C to stop\n")

        buffer = ""

        try:
            while True:
                if self.ser.in_waiting > 0:
                    data = self.ser.read_all().decode('utf-8', errors='ignore')
                    buffer += data

                    while 'SESSION_INFO_NTF:' in buffer:
                        start = buffer.find('SESSION_INFO_NTF:')
                        end = buffer.find('}', start)

                        if end != -1:
                            message = buffer[start:end+1]
                            buffer = buffer[end+1:]

                            # Parse distances
                            d1, d2, d3 = self.parse_distances(message)

                            # Add to filter buffers
                            if d1 is not None:
                                self.d1_buffer.append(d1)
                            if d2 is not None:
                                self.d2_buffer.append(d2)
                            if d3 is not None:
                                self.d3_buffer.append(d3)

                            # Get filtered distances
                            d1_f, d2_f, d3_f = self.get_filtered_distances()

                            # Count available distances
                            count, sources = self.count_available_distances(d1_f, d2_f, d3_f)

                            # Calculate position (now works with 2+ distances)
                            position = self.calculate_position(d1_f, d2_f, d3_f)

                            # Output results
                            if position and count >= 2:
                                x, y = position
                                print(f"Position: ({x:.1f}, {y:.1f})  |  {count} anchors: {', '.join(sources)}  |  d1={d1_f}, d2={d2_f}, d3={d3_f}")
                            elif count >= 2:
                                print(f"No valid position  |  {count} anchors: {', '.join(sources)}  |  d1={d1_f}, d2={d2_f}, d3={d3_f}")
                            else:
                                print(f"Insufficient data ({count} anchor{'s' if count != 1 else ''})  |  d1={d1_f}, d2={d2_f}, d3={d3_f}")
                        else:
                            break

                time.sleep(0.15)  # 150ms delay for SLAM bot

        except KeyboardInterrupt:
            print("\nStopping SLAM reader...")
        finally:
            self.ser.close()

def main():
    # Change port as needed
    PORT = '/dev/ttyACM0'  # Linux: /dev/ttyUSB0, Windows: COM3

    try:
        reader = UWBSLAMReader(PORT)
        reader.read_positions()
    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    main()
