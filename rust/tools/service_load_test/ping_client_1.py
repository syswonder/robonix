#!/usr/bin/env python3
"""
High-intensity ping pong service test client - Node 1

This node continuously calls the ping pong service to test concurrent access.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import time
import threading
import sys

try:
    from robonix_sdk.srv import PingPong
except ImportError:
    print("Warning: robonix_sdk.srv.PingPong not found. Make sure robonix_sdk is built and sourced.")
    print("Trying to use ros2 service call as fallback...")
    PingPong = None


class PingClient1(Node):
    """High-intensity ping pong test client node 1"""

    def __init__(self):
        super().__init__('ping_client_1')
        
        self.service_name = '/rbnx/ping'
        
        # Statistics
        self.total_requests = 0
        self.successful_requests = 0
        self.failed_requests = 0
        self.start_time = time.time()
        self.lock = threading.Lock()
        self.running = True
        
        # Create service client if service type is available
        if PingPong is not None:
            # Create QoS profile matching server configuration
            service_qos = QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                history=HistoryPolicy.KEEP_LAST,
                depth=1000,
                durability=DurabilityPolicy.VOLATILE
            )
            self.client = self.create_client(PingPong, self.service_name, qos_profile=service_qos)
            self.use_native_client = True
        else:
            self.client = None
            self.use_native_client = False
            # Fallback to ros2 service call
            import subprocess
            import json
            self.subprocess = subprocess
            self.json = json
        
        # Start high-intensity testing
        self.get_logger().info(f'Ping Client 1 starting high-intensity test on {self.service_name}')
        self.test_thread = threading.Thread(target=self._run_test, daemon=True)
        self.test_thread.start()
        
        # Print statistics periodically
        self.create_timer(5.0, self._print_stats)
    
    def _call_ping_service(self, sequence: int):
        """Call the ping pong service"""
        if self.use_native_client:
            return self._call_ping_service_native(sequence)
        else:
            return self._call_ping_service_subprocess(sequence)
    
    def _call_ping_service_native(self, sequence: int):
        """Call the ping pong service using native ROS2 client"""
        try:
            # Wait for service with longer timeout on first few calls
            wait_timeout = 5.0 if sequence < 10 else 0.1
            if not self.client.wait_for_service(timeout_sec=wait_timeout):
                return False, "Service not available"
            
            request = PingPong.Request()
            request.message = f'ping from client 1'
            request.sequence = sequence
            
            future = self.client.call_async(request)
            
            # Wait for response with timeout
            start_time = time.time()
            timeout_sec = 3.0  # Increased timeout for high load
            while not future.done() and (time.time() - start_time) < timeout_sec:
                rclpy.spin_once(self, timeout_sec=0.01)
            
            if future.done():
                try:
                    response = future.result()
                    return True, response
                except Exception as e:
                    return False, str(e)
            else:
                return False, "Timeout"
        except Exception as e:
            return False, str(e)
    
    def _call_ping_service_subprocess(self, sequence: int):
        """Call the ping pong service using ros2 service call (fallback)"""
        try:
            request_data = {
                'message': f'ping from client 1',
                'sequence': sequence
            }
            
            cmd = [
                'ros2', 'service', 'call',
                self.service_name,
                'robonix_sdk/srv/PingPong',
                self.json.dumps(request_data)
            ]
            
            result = self.subprocess.run(
                cmd,
                capture_output=True,
                text=True,
                timeout=2.0
            )
            
            if result.returncode == 0:
                return True, result.stdout
            else:
                return False, result.stderr
                
        except self.subprocess.TimeoutExpired:
            return False, "Timeout"
        except Exception as e:
            return False, str(e)
    
    def _run_test(self):
        """Run high-intensity test loop"""
        sequence = 0
        while self.running and rclpy.ok():
            sequence += 1
            success, result = self._call_ping_service(sequence)
            
            with self.lock:
                self.total_requests += 1
                if success:
                    self.successful_requests += 1
                    if sequence % 100 == 0:
                        self.get_logger().info(
                            f'Client 1: Sent {sequence} requests, '
                            f'success rate: {100.0 * self.successful_requests / self.total_requests:.1f}%'
                        )
                else:
                    self.failed_requests += 1
                    if self.failed_requests % 10 == 0:
                        self.get_logger().warn(
                            f'Client 1: Failed request #{self.failed_requests}, '
                            f'error: {result[:100] if result else "Unknown"}'
                        )
            
            # High intensity: minimal delay
            time.sleep(0.001)  # 1ms delay = ~1000 requests/second
    
    def _print_stats(self):
        """Print statistics"""
        elapsed = time.time() - self.start_time
        with self.lock:
            total = self.total_requests
            success = self.successful_requests
            failed = self.failed_requests
            rate = total / elapsed if elapsed > 0 else 0
            success_rate = 100.0 * success / total if total > 0 else 0
            
            self.get_logger().info(
                f'Client 1 Stats: Total={total}, Success={success}, Failed={failed}, '
                f'Rate={rate:.1f} req/s, Success Rate={success_rate:.1f}%'
            )
    
    def shutdown(self):
        """Shutdown the test"""
        self.running = False
        if self.test_thread.is_alive():
            self.test_thread.join(timeout=1.0)


def main(args=None):
    rclpy.init(args=args)
    node = None
    
    try:
        node = PingClient1()
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node:
            node.get_logger().info('Shutting down...')
            node.shutdown()
    except Exception as e:
        if node:
            node.get_logger().error(f'Error: {e}')
    finally:
        if node:
            try:
                node.destroy_node()
            except:
                pass
        try:
            rclpy.shutdown()
        except:
            pass  # Ignore if already shut down


if __name__ == '__main__':
    main()
