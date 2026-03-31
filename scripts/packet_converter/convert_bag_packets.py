#!/usr/bin/env python3
import argparse
import sys
import math
import numpy as np
import threading
import queue
import os
from concurrent.futures import ThreadPoolExecutor, as_completed
from collections import deque
from tqdm import tqdm
from typing import Optional, Tuple, Dict, Any

# ROS 2 Imports
try:
    import rclpy
    from rclpy.serialization import serialize_message, deserialize_message
    from rosbag2_py import (SequentialReader, StorageOptions, ConverterOptions, 
                            SequentialWriter, TopicMetadata)
    from sensor_msgs.msg import PointCloud2, Imu, PointField
    from std_msgs.msg import String, Header
    from ouster_sensor_msgs.msg import PacketMsg
except ImportError as e:
    print(f"Error: Missing ROS 2 dependencies: {e}")
    sys.exit(1)

# Ouster SDK Imports
try:
    from ouster.sdk.core import SensorInfo, XYZLut, LidarScan, ScanBatcher, PacketFormat, ChanField
    from ouster.sdk._bindings.client import LidarPacket, ImuPacket
except ImportError as e:
    print(f"Error: Ouster Python SDK not found: {e}")
    sys.exit(1)

class BagConverter:
    def __init__(self, args):
        self.args = args
        self.sensor_info = None
        self.xyzlut = None
        self.packet_format = None
        self.scan_batcher = None
        self.lidar_scan = None
        
        # Thread-safe counters
        self._lock = threading.Lock()
        self.imu_count = 0
        self.scan_count = 0
        self.packets_processed = 0
        
        self.pbar = None
        self.first_packet_timestamp = None
        self.update_interval = 100
        self.scan_height = 0
        self.ring_template = None
        self.imu_pkt = None
        self.imu_pkt_view = None
        self.lidar_pkt = None
        self.lidar_pkt_view = None
        
        # Threading components
        self.metadata_received = threading.Event()
        self.shutdown_event = threading.Event()
        
        # Queues for packet pipeline (with size limits for memory management)
        self.imu_queue = queue.Queue(maxsize=1000)  # Buffer IMU packets
        self.lidar_queue = queue.Queue(maxsize=200)  # Buffer lidar packets
        self.passthrough_queue = queue.Queue(maxsize=2000)  # Original data passthrough
        self.write_queue = queue.Queue(maxsize=1000)  # Pre-serialized messages ready to write
        
        # CPU count for worker threads
        self.num_workers = max(1, os.cpu_count() or 1)
        if self.num_workers > 4:
            self.num_workers = max(2, self.num_workers - 2)  # Reserve CPUs for I/O
        
        # Persistent thread pool for field extraction (avoid per-scan creation overhead)
        self._field_executor = None
        
        if getattr(self.args, 'progress_update_interval', 0) > 0:
            self.update_interval = self.args.progress_update_interval
        else:
            self.update_interval = 500  # Higher default reduces overhead
    
    @staticmethod
    def parse_size(size_str):
        """
        Parse size string to bytes.
        Examples: '500M', '1G', '100K', '1024' (bytes)
        Returns 0 if size_str is None, empty, or '0'
        """
        if not size_str or size_str == '0':
            return 0
        
        size_str = size_str.strip().upper()
        
        # Parse suffix
        multipliers = {
            'K': 1024,
            'M': 1024 * 1024,
            'G': 1024 * 1024 * 1024,
            'KB': 1024,
            'MB': 1024 * 1024,
            'GB': 1024 * 1024 * 1024,
        }
        
        for suffix, multiplier in multipliers.items():
            if size_str.endswith(suffix):
                try:
                    value = float(size_str[:-len(suffix)])
                    return int(value * multiplier)
                except ValueError:
                    raise ValueError(f"Invalid size format: {size_str}")
        
        # No suffix, assume bytes
        try:
            return int(size_str)
        except ValueError:
            raise ValueError(f"Invalid size format: {size_str}")
    
    @staticmethod
    def parse_duration(duration_str):
        """
        Parse duration string to nanoseconds.
        Examples: '60s', '5m', '1h', '30' (seconds)
        Returns 0 if duration_str is None, empty, or '0'
        """
        if not duration_str or duration_str == '0':
            return 0
        
        duration_str = duration_str.strip().lower()
        
        # Parse suffix
        multipliers = {
            's': 1,
            'm': 60,
            'h': 3600,
            'sec': 1,
            'min': 60,
            'hour': 3600,
        }
        
        for suffix, multiplier in multipliers.items():
            if duration_str.endswith(suffix):
                try:
                    value = float(duration_str[:-len(suffix)])
                    return int(value * multiplier * 1e9)  # Convert to nanoseconds
                except ValueError:
                    raise ValueError(f"Invalid duration format: {duration_str}")
        
        # No suffix, assume seconds
        try:
            return int(float(duration_str) * 1e9)
        except ValueError:
            raise ValueError(f"Invalid duration format: {duration_str}")

    def run(self):
        """Main orchestrator for threaded bag conversion."""
        # 1. Setup Reader
        reader = SequentialReader()
        reader.open(
            StorageOptions(uri=self.args.input_bag, storage_id='mcap'),
            ConverterOptions('', '')
        )

        # 2. Get metadata from input bag
        bag_metadata = reader.get_metadata()
        duration_ns = bag_metadata.duration.nanoseconds if hasattr(bag_metadata.duration, 'nanoseconds') else 0
        duration_sec = duration_ns / 1e9
        
        print(f"Input bag info:")
        print(f"  - Storage ID: {bag_metadata.storage_identifier}")
        print(f"  - Duration: {duration_sec:.2f} seconds")
        print(f"  - Message count: {bag_metadata.message_count}")
        print(f"  - Files: {len(bag_metadata.relative_file_paths)}")
        print(f"  - Using {self.num_workers} worker threads for CPU-intensive tasks")

        # 3. Setup Writer
        writer = SequentialWriter()
        
        # Detect output format from extension or explicit argument
        output_format = self.args.output_format
        if output_format is None:
            if self.args.output_bag.endswith('.bag'):
                output_format = 'sqlite3'  # ROS1 bag format
            else:
                output_format = 'mcap'  # ROS2 MCAP format
        
        output_storage_options = StorageOptions(
            uri=self.args.output_bag,
            storage_id=output_format
        )

        max_bagfile_size = self.parse_size(self.args.max_bagfile_size)
        max_bagfile_duration = self.parse_duration(self.args.max_bagfile_duration)

        if max_bagfile_size:
            output_storage_options.max_bagfile_size = int(max_bagfile_size)
        if max_bagfile_duration:
            output_storage_options.max_bagfile_duration = int(max_bagfile_duration / 1e9)

        if max_bagfile_size > 0 or max_bagfile_duration > 0:
            print(f"Output bag split settings:")
            if max_bagfile_size > 0:
                print(f"  - max_bagfile_size: {max_bagfile_size / (1024*1024):.2f} MB")
            if max_bagfile_duration > 0:
                print(f"  - max_bagfile_duration: {max_bagfile_duration / 1e9:.2f} seconds")
        else:
            print(f"Output bag split settings: No splitting (single file)")
        
        writer.open(output_storage_options, ConverterOptions('', ''))

        # 4. Transfer Topic Metadata
        all_reader_topics = reader.get_all_topics_and_types()
        input_qos_map = {t.name: t.offered_qos_profiles for t in all_reader_topics}

        if not self.args.skip_original_topics:
            for topic_meta in all_reader_topics:
                writer.create_topic(topic_meta)

        def create_safe_metadata(name, type_str, qos_profiles):
            return TopicMetadata(
                0, name, type_str, 'cdr',
                qos_profiles if qos_profiles else []
            )

        pc_qos = input_qos_map.get(self.args.lidar_packets_topic, [])
        writer.create_topic(create_safe_metadata(self.args.points_topic, 'sensor_msgs/msg/PointCloud2', pc_qos))

        imu_qos = input_qos_map.get(self.args.imu_packets_topic, [])
        writer.create_topic(create_safe_metadata(self.args.imu_topic, 'sensor_msgs/msg/Imu', imu_qos))
        
        print(f"Processing bag: {self.args.input_bag}")
        self.pbar = tqdm(unit="pkts", desc="Processing", dynamic_ncols=True)

        # Load metadata file if provided
        try:
            meta_file = getattr(self.args, 'metadata_file', None)
            if meta_file and os.path.exists(meta_file):
                with open(meta_file, 'r') as mf:
                    meta_str = mf.read()
                self.initialize_sensor_from_string(meta_str)
                print(f"[DEBUG] Initialized sensor from metadata file: {meta_file}")
                self.metadata_received.set()
        except Exception as e:
            print(f"[DEBUG] Error loading metadata_file: {e}")

        # Start worker threads
        threads = []
        
        # IMU worker threads (can run in parallel)
        imu_threads = []
        for i in range(min(2, self.num_workers)):  # 2 IMU workers
            t = threading.Thread(
                target=self._imu_worker,
                args=(i,),
                daemon=True,
                name=f"IMUWorker-{i}"
            )
            t.start()
            imu_threads.append(t)
            threads.append(t)
        
        # Single lidar processor (stateful scan_batcher)
        lidar_thread = threading.Thread(
            target=self._lidar_processor,
            daemon=True,
            name="LidarProcessor"
        )
        lidar_thread.start()
        threads.append(lidar_thread)
        
        # Writer thread (maintains ordering)
        writer_thread = threading.Thread(
            target=self._writer_worker,
            args=(writer,),
            daemon=True,
            name="WriterThread"
        )
        writer_thread.start()
        threads.append(writer_thread)
        
        # Passthrough writer thread (writes original data)
        if not self.args.skip_original_topics:
            passthrough_thread = threading.Thread(
                target=self._passthrough_writer,
                args=(writer,),
                daemon=True,
                name="PassthroughThread"
            )
            passthrough_thread.start()
            threads.append(passthrough_thread)

        # Reader thread (main data source)
        try:
            iteration_count = 0
            topic_counts = {}
            while reader.has_next():
                iteration_count += 1
                if iteration_count <= 10 or iteration_count % 5000 == 0:
                    print(f"[DEBUG] Iteration {iteration_count}, sensor_info={self.sensor_info is not None}")

                topic, data, tstamp = reader.read_next()

                if topic not in topic_counts:
                    topic_counts[topic] = 0
                    print(f"[DEBUG] Found topic: {topic}")
                topic_counts[topic] += 1
                
                # Route data to appropriate queues
                if not self.args.skip_original_topics:
                    self.passthrough_queue.put((topic, data, tstamp))
                
                if topic == self.args.metadata_topic:
                    self.handle_metadata(data)
                    self.metadata_received.set()  # Signal that metadata is ready
                elif topic == self.args.imu_packets_topic:
                    # Wait briefly for sensor to initialize
                    self.metadata_received.wait(timeout=30)
                    if self.sensor_info:
                        self.imu_queue.put((data, tstamp))
                elif topic == self.args.lidar_packets_topic:
                    self.metadata_received.wait(timeout=30)
                    if self.sensor_info:
                        self.lidar_queue.put((data, tstamp))
            
            # Signal workers that reading is complete
            print("[DEBUG] Reader finished, sending poison pills to workers...")
            self.shutdown_event.set()
            
            # Send sentinel values to queues
            for _ in range(2):  # 2 IMU workers
                self.imu_queue.put(None)
            self.lidar_queue.put(None)
            self.passthrough_queue.put(None)
            self.write_queue.put(None)
            
            # Wait for all threads to finish
            for t in threads:
                t.join(timeout=60)
                if t.is_alive():
                    print(f"[WARN] Thread {t.name} did not finish within timeout")

        except Exception as e:
            print(f"[ERROR] Reader thread error: {e}")
            self.shutdown_event.set()
            import traceback
            traceback.print_exc()
        finally:
            self.pbar.close()
            if self._field_executor:
                self._field_executor.shutdown(wait=False)
            writer.close()
            print(f"\nFinished. Scans: {self.scan_count}, IMU: {self.imu_count}")
    
    def _imu_worker(self, worker_id: int):
        """Worker thread for parallel IMU processing. Pre-serializes messages."""
        local_count = 0
        try:
            while not self.shutdown_event.is_set():
                try:
                    item = self.imu_queue.get(timeout=1)
                    if item is None:  # Poison pill
                        break
                    
                    data, tstamp = item
                    imu_msg, ts = self.process_imu(data, tstamp)
                    
                    if imu_msg is not None:
                        # Pre-serialize in worker thread to offload writer
                        serialized = serialize_message(imu_msg)
                        self.write_queue.put(('imu', self.args.imu_topic, serialized, ts))
                        local_count += 1
                        if local_count % self.update_interval == 0:
                            with self._lock:
                                self.packets_processed += self.update_interval
                            self.pbar.update(self.update_interval)
                            self.update_progress()
                except queue.Empty:
                    continue
                except Exception as e:
                    print(f"[ERROR] IMU Worker-{worker_id} error: {e}")
                    import traceback
                    traceback.print_exc()
        except Exception as e:
            print(f"[ERROR] IMU Worker-{worker_id} fatal error: {e}")
        finally:
            # Flush remaining count
            remainder = local_count % self.update_interval
            if remainder:
                with self._lock:
                    self.packets_processed += remainder
                self.pbar.update(remainder)

    def _lidar_processor(self):
        """Single-threaded lidar processor (maintains scan_batcher state). Pre-serializes messages."""
        local_count = 0
        try:
            while not self.shutdown_event.is_set():
                try:
                    item = self.lidar_queue.get(timeout=1)
                    if item is None:  # Poison pill
                        break
                    
                    data, tstamp = item
                    pc_msgs = self.process_lidar(data, tstamp)
                    
                    if pc_msgs:
                        for topic, msg, ts in pc_msgs:
                            # Pre-serialize in processor thread
                            serialized = serialize_message(msg)
                            self.write_queue.put(('pointcloud', topic, serialized, ts))
                            local_count += 1
                            if local_count % self.update_interval == 0:
                                with self._lock:
                                    self.packets_processed += self.update_interval
                                self.pbar.update(self.update_interval)
                                self.update_progress()
                except queue.Empty:
                    continue
                except Exception as e:
                    print(f"[ERROR] Lidar processor error: {e}")
                    import traceback
                    traceback.print_exc()
        except Exception as e:
            print(f"[ERROR] Lidar processor fatal error: {e}")
        finally:
            remainder = local_count % self.update_interval
            if remainder:
                with self._lock:
                    self.packets_processed += remainder
                self.pbar.update(remainder)

    def _writer_worker(self, writer):
        """Single-threaded writer. Receives pre-serialized data for minimal latency."""
        try:
            while not self.shutdown_event.is_set():
                try:
                    item = self.write_queue.get(timeout=1)
                    if item is None:  # Poison pill
                        break
                    
                    msg_type, topic, serialized_data, tstamp = item
                    # Data is already serialized by worker threads
                    writer.write(topic, serialized_data, tstamp)
                    if msg_type == 'imu':
                        self.imu_count += 1  # Single writer, no lock needed
                    elif msg_type == 'pointcloud':
                        self.scan_count += 1
                except queue.Empty:
                    continue
                except Exception as e:
                    print(f"[ERROR] Writer error: {e}")
                    import traceback
                    traceback.print_exc()
        except Exception as e:
            print(f"[ERROR] Writer fatal error: {e}")

    def _passthrough_writer(self, writer):
        """Write original topic data to preserve all information."""
        try:
            while not self.shutdown_event.is_set():
                try:
                    item = self.passthrough_queue.get(timeout=1)
                    if item is None:  # Poison pill
                        break
                    
                    topic, data, tstamp = item
                    writer.write(topic, data, tstamp)
                except queue.Empty:
                    continue
                except Exception as e:
                    print(f"[ERROR] Passthrough writer error: {e}")
        except Exception as e:
            print(f"[ERROR] Passthrough writer fatal error: {e}")

    def handle_metadata(self, data):
        try:
            meta_msg = deserialize_message(data, String)
            print(f"[DEBUG] Metadata received, length: {len(meta_msg.data)}")
            # Delegate to initializer so metadata can also be loaded from file
            self.initialize_sensor_from_string(meta_msg.data)
            if self.sensor_info:
                print(f"[DEBUG] SensorInfo initialized: {self.sensor_info.prod_line}")
            
            # Check what fields are available by testing if we can access them
            # Note: TIMESTAMP and RING are always extracted (always present in packet)
            self.has_signal = False
            self.has_reflectivity = False
            self.has_range = False
            self.has_ambient = False
            
            try:
                if hasattr(ChanField, 'SIGNAL'):
                    self.lidar_scan.field(ChanField.SIGNAL)
                    self.has_signal = True
            except:
                pass
            
            try:
                if hasattr(ChanField, 'REFLECTIVITY'):
                    self.lidar_scan.field(ChanField.REFLECTIVITY)
                    self.has_reflectivity = True
            except:
                pass
            
            try:
                if hasattr(ChanField, 'RANGE'):
                    self.lidar_scan.field(ChanField.RANGE)
                    self.has_range = True
            except:
                pass
            
            try:
                if hasattr(ChanField, 'NEAR_IR'):
                    self.lidar_scan.field(ChanField.NEAR_IR)
                    self.has_ambient = True
            except:
                pass
            
            fields_info = ["TIMESTAMP", "RING"]  # Always present
            if self.has_signal:
                fields_info.append("SIGNAL")
            if self.has_reflectivity:
                fields_info.append("REFLECTIVITY")
            if self.has_ambient:
                fields_info.append("AMBIENT")
            if self.has_range:
                fields_info.append("RANGE")
            fields_str = ", ".join(fields_info)
            
            self.pbar.write(f"[*] Ouster Metadata Initialized: {self.sensor_info.prod_line}")
            self.pbar.write(f"[*] Available fields: {fields_str}")
            self.pbar.write(f"[*] LidarScan info: {self.lidar_scan}")
        except Exception as e:
            error_msg = f"[!] Metadata Error: {e}"
            print(error_msg)  # Print to stdout immediately
            if self.pbar:
                self.pbar.write(error_msg)

    def decode_packet_msg(self, data):
        """Deserialize a packet once and return (header_timestamp_ns, raw_buffer)."""
        try:
            pkt_msg = deserialize_message(data, PacketMsg)
            stamp = pkt_msg.header.stamp if hasattr(pkt_msg, 'header') else None
            timestamp_ns = 0 if stamp is None else (stamp.sec * 1_000_000_000 + stamp.nanosec)
            try:
                return timestamp_ns, memoryview(pkt_msg.buf)
            except TypeError:
                return timestamp_ns, bytes(pkt_msg.buf)
        except Exception:
            return 0, data

    def initialize_sensor_from_string(self, meta_str):
        """Initialize SensorInfo and related helpers from a metadata string.
        This mirrors what `handle_metadata` does after deserializing the std_msgs/String.
        """
        try:
            self.sensor_info = SensorInfo(meta_str)
            self.xyzlut = XYZLut(self.sensor_info)
            self.packet_format = PacketFormat(self.sensor_info)
            self.scan_batcher = ScanBatcher(self.sensor_info)

            w = self.sensor_info.format.columns_per_frame
            h = self.sensor_info.format.pixels_per_column
            self.scan_height = h
            self.lidar_scan = LidarScan(h, w)
            self.ring_template = np.repeat(np.arange(h, dtype=np.uint16), w)
            self.imu_pkt = ImuPacket(self.packet_format.imu_packet_size)
            self.imu_pkt_view = np.frombuffer(self.imu_pkt.buf, dtype=np.uint8)
            self.lidar_pkt = LidarPacket(self.packet_format.lidar_packet_size)
            self.lidar_pkt_view = np.frombuffer(self.lidar_pkt.buf, dtype=np.uint8)

            # Check fields availability
            self.has_signal = False
            self.has_reflectivity = False
            self.has_range = False
            self.has_ambient = False
            try:
                if hasattr(ChanField, 'SIGNAL'):
                    self.lidar_scan.field(ChanField.SIGNAL)
                    self.has_signal = True
            except:
                pass
            try:
                if hasattr(ChanField, 'REFLECTIVITY'):
                    self.lidar_scan.field(ChanField.REFLECTIVITY)
                    self.has_reflectivity = True
            except:
                pass
            try:
                if hasattr(ChanField, 'RANGE'):
                    self.lidar_scan.field(ChanField.RANGE)
                    self.has_range = True
            except:
                pass
            try:
                if hasattr(ChanField, 'NEAR_IR'):
                    self.lidar_scan.field(ChanField.NEAR_IR)
                    self.has_ambient = True
            except:
                pass
        except Exception as e:
            print(f"[ERROR] initialize_sensor_from_string failed: {e}")

    @staticmethod
    def fill_stamp(stamp_msg, timestamp_ns):
        stamp_msg.sec = timestamp_ns // 1_000_000_000
        stamp_msg.nanosec = timestamp_ns % 1_000_000_000

    def update_progress(self):
        """Update progress postfix with current stats."""
        self.pbar.set_postfix(scans=self.scan_count, imu=self.imu_count)

    def create_cloud_with_fields(self, header, points, additional_fields):
        """
        Create a PointCloud2 message with XYZ and additional fields.
        Fields structure with padding:
        - x, y, z: FLOAT32 at offsets 0, 4, 8
        - [4 byte padding]
        - intensity: FLOAT32 at offset 16
        - t: UINT32 at offset 20
        - reflectivity: UINT16 at offset 24
        - ring: UINT16 at offset 26
        - ambient: UINT16 at offset 28
        - [2 byte padding]
        - range: UINT32 at offset 32
        - [12 byte padding to reach 48]
        
        Args:
            header: ROS Header
            points: Nx3 numpy array of XYZ coordinates
            additional_fields: Dict mapping field_name to numpy_array
        
        Returns:
            PointCloud2 message
        """
        # Define fields with correct offsets and datatypes
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=16, datatype=PointField.FLOAT32, count=1),
            PointField(name='t', offset=20, datatype=PointField.UINT32, count=1),
            PointField(name='reflectivity', offset=24, datatype=PointField.UINT16, count=1),
            PointField(name='ring', offset=26, datatype=PointField.UINT16, count=1),
            PointField(name='ambient', offset=28, datatype=PointField.UINT16, count=1),
            PointField(name='range', offset=32, datatype=PointField.UINT32, count=1),
        ]
        
        # Create structured array matching exact offsets
        dtype_list = [
            ('x', np.float32),
            ('y', np.float32),
            ('z', np.float32),
            ('pad1', np.uint32),  # 4 bytes padding (offsets 12-15)
            ('intensity', np.float32),  # offset 16-19
            ('t', np.uint32),  # offset 20-23
            ('reflectivity', np.uint16),  # offset 24-25
            ('ring', np.uint16),  # offset 26-27
            ('ambient', np.uint16),  # offset 28-29
            ('pad2', np.uint16),  # 2 bytes padding (offsets 30-31)
            ('range', np.uint32),  # offset 32-35
            ('pad3', np.uint32),  # 4 bytes padding (offsets 36-39)
            ('pad4', np.uint32),  # 4 bytes padding (offsets 40-43)
            ('pad5', np.uint32),  # 4 bytes padding (offsets 44-47)
        ]
        
        cloud_data = np.zeros(len(points), dtype=dtype_list)
        cloud_data['x'] = points[:, 0]
        cloud_data['y'] = points[:, 1]
        cloud_data['z'] = points[:, 2]
        
        # Fill in optional fields if present
        if 'intensity' in additional_fields:
            cloud_data['intensity'] = additional_fields['intensity'].astype(np.float32)
        
        if 't' in additional_fields:
            cloud_data['t'] = additional_fields['t'].astype(np.uint32)
        
        if 'reflectivity' in additional_fields:
            cloud_data['reflectivity'] = additional_fields['reflectivity'].astype(np.uint16)
        
        if 'ring' in additional_fields:
            cloud_data['ring'] = additional_fields['ring'].astype(np.uint16)
        
        if 'ambient' in additional_fields:
            cloud_data['ambient'] = additional_fields['ambient'].astype(np.uint16)
        
        if 'range' in additional_fields:
            cloud_data['range'] = additional_fields['range'].astype(np.uint32)
        
        # Fixed point_step: 48 bytes
        point_step = 48
        
        # Create PointCloud2 message
        pc_msg = PointCloud2()
        pc_msg.header = header
        pc_msg.height = 1
        pc_msg.width = len(points)
        pc_msg.fields = fields
        pc_msg.is_bigendian = False
        pc_msg.point_step = point_step
        pc_msg.row_step = point_step * len(points)
        pc_msg.is_dense = True
        pc_msg.data = cloud_data.tobytes()
        
        return pc_msg

    def process_imu(self, data, tstamp) -> Tuple[Optional[Imu], int]:
        """
        Process IMU packet. Returns (message, timestamp) or (None, 0) on error.
        Thread-safe version - creates local packet buffer for each call.
        """
        try:
            # Extract timestamp from packet header
            header_timestamp, raw_buf = self.decode_packet_msg(data)
            msg_stamp = header_timestamp if header_timestamp != 0 else tstamp
            
            # Create thread-local packet buffer to avoid race conditions
            local_pkt = ImuPacket(self.packet_format.imu_packet_size)
            local_pkt_view = np.frombuffer(local_pkt.buf, dtype=np.uint8)
            
            src = np.frombuffer(raw_buf, dtype=np.uint8)
            copy_len = min(src.size, local_pkt_view.size)
            local_pkt_view[:copy_len] = src[:copy_len]
            
            imu_msg = Imu()
            self.fill_stamp(imu_msg.header.stamp, msg_stamp)
            imu_msg.header.frame_id = self.args.imu_frame_id
            
            imu_msg.linear_acceleration.x = self.packet_format.imu_la_x(local_pkt.buf) * 9.80665
            imu_msg.linear_acceleration.y = self.packet_format.imu_la_y(local_pkt.buf) * 9.80665
            imu_msg.linear_acceleration.z = self.packet_format.imu_la_z(local_pkt.buf) * 9.80665
            imu_msg.angular_velocity.x = math.radians(self.packet_format.imu_av_x(local_pkt.buf))
            imu_msg.angular_velocity.y = math.radians(self.packet_format.imu_av_y(local_pkt.buf))
            imu_msg.angular_velocity.z = math.radians(self.packet_format.imu_av_z(local_pkt.buf))

            return imu_msg, tstamp
        except Exception as e:
            print(f"[ERROR] IMU processing error: {e}")
            import traceback
            traceback.print_exc()
            return None, 0

    def process_lidar(self, data, tstamp) -> list:
        """
        Process lidar packet. Returns list of (topic, message, timestamp) tuples.
        Only returns data when a complete scan is batched.
        Single-threaded due to scan_batcher statefulness - reuses instance buffer.
        """
        try:
            if not self.scan_batcher:
                print("[ERROR] scan_batcher not initialized!")
                return []
            
            # Extract timestamp from packet header 
            header_timestamp, raw_buf = self.decode_packet_msg(data)
            msg_stamp = header_timestamp if header_timestamp != 0 else tstamp
            
            # Reuse instance buffer (single-threaded, no race condition)
            src = np.frombuffer(raw_buf, dtype=np.uint8)
            copy_len = min(src.size, self.lidar_pkt_view.size)
            self.lidar_pkt_view[:copy_len] = src[:copy_len]

            # Capture first packet timestamp when starting a new batch
            if self.first_packet_timestamp is None:
                self.first_packet_timestamp = msg_stamp

            # Check if batch is complete - this MUST be sequential
            batch_complete = self.scan_batcher(self.lidar_pkt, self.lidar_scan)
            if not batch_complete:
                return []
            
            # Batch is complete - process it (can parallelize field extraction)
            try:
                xyz = self.xyzlut(self.lidar_scan)
                points = xyz.reshape(-1, 3)
                mask = np.any(points != 0, axis=1)
                valid_points = points[mask]

                if valid_points.size == 0:
                    self.first_packet_timestamp = None
                    return []

                header = Header()
                self.fill_stamp(header.stamp, self.first_packet_timestamp)
                header.frame_id = self.args.lidar_frame_id
                
                # Extract fields in parallel if possible
                additional_fields = self._extract_lidar_fields(mask)
                
                # Create point cloud
                pc_msg = self.create_cloud_with_fields(header, valid_points, additional_fields)
                
                self.first_packet_timestamp = None
                return [(self.args.points_topic, pc_msg, tstamp)]
            
            except Exception as e:
                print(f"[ERROR] Lidar field extraction error: {e}")
                import traceback
                traceback.print_exc()
                self.first_packet_timestamp = None
                return []
        
        except Exception as e:
            print(f"[ERROR] Lidar processing error: {e}")
            import traceback
            traceback.print_exc()
            return []
    
    def _extract_lidar_fields(self, mask: np.ndarray) -> Dict[str, np.ndarray]:
        """
        Extract lidar fields in parallel where possible.
        Parallelizable operations are done via ThreadPoolExecutor.
        """
        additional_fields = {}
        
        def extract_timestamp():
            try:
                column_timestamps_ns = self.lidar_scan.timestamp
                column_timestamps_ns_int64 = column_timestamps_ns.astype(np.int64)
                ref_int64 = column_timestamps_ns_int64[0] if column_timestamps_ns_int64.size else np.int64(0)
                relative_timestamps_ns = column_timestamps_ns_int64 - ref_int64
                relative_timestamps_ns = relative_timestamps_ns.astype(np.uint32)
                timestamp_flat = np.tile(relative_timestamps_ns, self.scan_height)[mask]
                return 't', timestamp_flat
            except Exception as e:
                self.pbar.write(f"[!] Failed to extract TIMESTAMP: {e}")
                return None, None
        
        def extract_ring():
            try:
                ring_flat = self.ring_template[mask]
                return 'ring', ring_flat.astype(np.uint16)
            except Exception as e:
                self.pbar.write(f"[!] Failed to extract RING: {e}")
                return None, None
        
        def extract_signal():
            if not self.has_signal:
                return None, None
            try:
                signal = self.lidar_scan.field(ChanField.SIGNAL)
                signal_flat = signal.reshape(-1)[mask]
                return 'intensity', signal_flat.astype(np.float32)
            except Exception as e:
                self.pbar.write(f"[!] Failed to extract SIGNAL: {e}")
                return None, None
        
        def extract_reflectivity():
            if not self.has_reflectivity:
                return None, None
            try:
                reflectivity = self.lidar_scan.field(ChanField.REFLECTIVITY)
                reflectivity_flat = reflectivity.reshape(-1)[mask]
                return 'reflectivity', reflectivity_flat.astype(np.uint16)
            except Exception as e:
                self.pbar.write(f"[!] Failed to extract REFLECTIVITY: {e}")
                return None, None
        
        def extract_ambient():
            if not self.has_ambient:
                return None, None
            try:
                near_ir = self.lidar_scan.field(ChanField.NEAR_IR)
                near_ir_flat = near_ir.reshape(-1)[mask]
                return 'ambient', near_ir_flat.astype(np.uint16)
            except Exception as e:
                self.pbar.write(f"[!] Failed to extract NEAR_IR: {e}")
                return None, None
        
        def extract_range():
            if not self.has_range:
                return None, None
            try:
                range_data = self.lidar_scan.field(ChanField.RANGE)
                range_flat = range_data.reshape(-1)[mask]
                return 'range', range_flat.astype(np.uint32)
            except Exception as e:
                self.pbar.write(f"[!] Failed to extract RANGE: {e}")
                return None, None
        
        # Use persistent ThreadPoolExecutor (created once, reused per scan)
        if self._field_executor is None:
            self._field_executor = ThreadPoolExecutor(max_workers=min(4, self.num_workers))
        
        futures = {
            self._field_executor.submit(extract_timestamp): 'timestamp',
            self._field_executor.submit(extract_ring): 'ring',
            self._field_executor.submit(extract_signal): 'signal',
            self._field_executor.submit(extract_reflectivity): 'reflectivity',
            self._field_executor.submit(extract_ambient): 'ambient',
            self._field_executor.submit(extract_range): 'range',
        }
        
        for future in as_completed(futures):
            try:
                field_name, field_data = future.result()
                if field_name is not None and field_data is not None:
                    additional_fields[field_name] = field_data
            except Exception as e:
                print(f"[ERROR] Field extraction future failed: {e}")
        
        return additional_fields

if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description='Ouster Bag Converter - ROS2 Header Timestamps',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog='''
Examples:
  # No splitting
  %(prog)s -i input.mcap -o output.mcap
  
  # Split by size
  %(prog)s -i input.mcap -o output.mcap --max-bagfile-size 500M
  %(prog)s -i input.mcap -o output.mcap --max-bagfile-size 2G
  
  # Split by duration
  %(prog)s -i input.mcap -o output.mcap --max-bagfile-duration 60s
  %(prog)s -i input.mcap -o output.mcap --max-bagfile-duration 5m
  
  # Split by both (whichever limit is reached first)
  %(prog)s -i input.mcap -o output.mcap --max-bagfile-size 500M --max-bagfile-duration 60s

Size suffixes: K/KB (kilobytes), M/MB (megabytes), G/GB (gigabytes)
Duration suffixes: s/sec (seconds), m/min (minutes), h/hour (hours)
        '''
    )
    parser.add_argument('--input-bag', '-i', required=True, help='Input bag file path (MCAP format)')
    parser.add_argument('--output-bag', '-o', required=True, help='Output bag file path (.mcap for ROS2 MCAP, .bag for ROS1 bag format)')
    parser.add_argument('--output-format', default=None, help='Force output format (mcap or sqlite3 for ROS1 bags, auto-detected from extension if absent)')
    parser.add_argument('--metadata-topic', default='/ouster/metadata', help='Metadata topic name')
    parser.add_argument('--lidar-packets-topic', default='/ouster/lidar_packets', help='Lidar packets topic name')
    parser.add_argument('--imu-packets-topic', default='/ouster/imu_packets', help='IMU packets topic name')
    parser.add_argument('--points-topic', default='/ouster/points', help='Output points topic name')
    parser.add_argument('--imu-topic', default='/ouster/imu', help='Output IMU topic name')
    parser.add_argument('--lidar-frame-id', default='os_lidar', help='Frame ID for lidar data')
    parser.add_argument('--imu-frame-id', default='os_imu', help='Frame ID for IMU data')
    parser.add_argument('--max-bagfile-size', type=str, default='0', 
                        help='Maximum bag file size (0 = no splitting). Examples: 500M, 2G, 1024K')
    parser.add_argument('--max-bagfile-duration', type=str, default='0',
                        help='Maximum bag file duration (0 = no splitting). Examples: 60s, 5m, 1h')
    parser.add_argument('--skip-original-topics', action='store_true',
                        help='Do not copy input topics to output (faster, only writes converted topics)')
    parser.add_argument('--progress-update-interval', type=int, default=500,
                        help='Update progress every N packets (higher values reduce progress overhead)')
    parser.add_argument('--metadata-file', type=str, default=None,
                        help='Path to a file containing /ouster/metadata payload to initialize the converter')
    
    args = parser.parse_args()
    rclpy.init()
    try:
        BagConverter(args).run()
    finally:
        rclpy.shutdown()
