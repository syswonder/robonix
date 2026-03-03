#!/usr/bin/env python3
"""
Minimal test script to query primitive service
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy,
    LivelinessPolicy
)
from rclpy.duration import Duration
import time
import sys

try:
    from robonix_sdk.srv import QueryPrimitive
except ImportError:
    print("ERROR: robonix_sdk.srv.QueryPrimitive not found")
    print("Make sure robonix-sdk is built and sourced")
    sys.exit(1)


class QueryPrimitiveTest(Node):
    """Minimal test node to query primitive service"""

    def __init__(self):
        super().__init__('query_primitive_test')
        
        # Create QoS profile matching server configuration exactly
        # Server uses: KeepLast(1000), Reliable, Volatile, INFINITE deadline/lifespan, Automatic liveliness
        service_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1000,
            durability=DurabilityPolicy.VOLATILE
        )
        # Set additional QoS policies to match server (INFINITE = Duration(seconds=0))
        service_qos.deadline = Duration(seconds=0)  # INFINITE
        service_qos.lifespan = Duration(seconds=0)  # INFINITE
        service_qos.liveliness = LivelinessPolicy.AUTOMATIC
        service_qos.liveliness_lease_duration = Duration(seconds=0)  # INFINITE
        
        self.client = self.create_client(
            QueryPrimitive,
            '/rbnx/prm/query',
            qos_profile=service_qos
        )
        
        self.get_logger().info('QueryPrimitive client created')
    
    def test_query(self):
        """Test querying prm::camera.rgb"""
        import json
        
        self.get_logger().info('Waiting for service /rbnx/prm/query...')
        
        # Wait for service - spin while waiting to help discovery
        service_available = False
        start_wait = time.time()
        timeout_sec = 15.0  # Total timeout for service discovery
        
        while (time.time() - start_wait) < timeout_sec:
            if self.client.wait_for_service(timeout_sec=1.0):
                service_available = True
                break
            # Spin to help discovery
            rclpy.spin_once(self, timeout_sec=0.1)
        
        if not service_available:
            self.get_logger().error(f'Service /rbnx/prm/query not available after {timeout_sec}s')
            return False
        
        self.get_logger().info('Service available, making request...')
        
        # Create request
        request = QueryPrimitive.Request()
        request.name = 'prm::camera.rgb'
        filter_dict = {"camera": "front"}
        request.filter = json.dumps(filter_dict)
        
        self.get_logger().info(f'Request: name={request.name}, filter={request.filter}')
        
        # Call service
        future = self.client.call_async(request)
        self.get_logger().info('Service call sent, waiting for response...')
        
        # Wait for response with longer timeout
        start_time = time.time()
        timeout_sec = 10.0  # Increased timeout
        spin_count = 0
        while not future.done() and (time.time() - start_time) < timeout_sec:
            rclpy.spin_once(self, timeout_sec=0.01)
            spin_count += 1
            if spin_count % 100 == 0:
                elapsed = time.time() - start_time
                self.get_logger().info(f'  Still waiting... ({elapsed:.1f}s elapsed)')
        
        if not future.done():
            elapsed = time.time() - start_time
            self.get_logger().error(f'Service call timeout after {elapsed:.1f}s')
            # Try to cancel
            try:
                future.cancel()
            except:
                pass
            return False
        
        try:
            response = future.result()
            self.get_logger().info(f'Service call successful!')
            self.get_logger().info(f'Response: {len(response.instances)} instance(s) found')
            
            if response.instances:
                instance = response.instances[0]
                self.get_logger().info(f'  Provider: {instance.provider}')
                self.get_logger().info(f'  Version: {instance.version}')
                self.get_logger().info(f'  Output schema: {instance.output_schema}')
                return True
            else:
                self.get_logger().warn('  No instances found')
                return False
                
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')
            import traceback
            self.get_logger().error(f'Traceback:\n{traceback.format_exc()}')
            return False


def main(args=None):
    rclpy.init(args=args)
    node = None
    
    try:
        node = QueryPrimitiveTest()
        success = node.test_query()
        if success:
            print("SUCCESS: Service call completed")
            sys.exit(0)
        else:
            print("FAILED: Service call failed")
            sys.exit(1)
    except Exception as e:
        print(f"ERROR: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
    finally:
        if node:
            try:
                node.destroy_node()
            except:
                pass
        try:
            rclpy.shutdown()
        except:
            pass


if __name__ == '__main__':
    main()

