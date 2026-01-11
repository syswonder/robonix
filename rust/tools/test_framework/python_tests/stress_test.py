#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import time, sys, argparse, json

try:
    from robonix_sdk.srv import SubmitTask
except ImportError:
    print("Error: robonix_sdk not found.")
    sys.exit(1)

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--client-id', type=int, default=0)
    parser.add_argument('--requests', type=int, default=1000)
    parser.add_argument('--rate', type=int, default=100)
    parser.add_argument('--duration', type=int, default=0)
    args = parser.parse_args()
    
    rclpy.init()
    node = Node(f'stress_test_python_{args.client_id}')
    qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=10, durability=DurabilityPolicy.VOLATILE)
    client = node.create_client(SubmitTask, '/rbnx/task/submit', qos_profile=qos)
    
    if not client.wait_for_service(timeout_sec=10.0):
        print("Service not available")
        return

    stats = {'success': 0, 'failed': 0, 'latencies': []}
    interval = 1.0 / args.rate if args.rate > 0 else 0
    start_time = time.time()
    end_time = start_time + args.duration if args.duration > 0 else None
    last_req = start_time

    for i in range(args.requests):
        if (end_time and time.time() >= end_time): break
        elapsed = time.time() - last_req
        if elapsed < interval: time.sleep(interval - elapsed)
        last_req = time.time()
        
        req = SubmitTask.Request(description=f'task {i}', params='{}')
        call_start = time.time()
        
        future = client.call_async(req)
        while rclpy.ok() and not future.done() and (time.time() - call_start) < 5.0:
            rclpy.spin_once(node, timeout_sec=0.01)
            
        if future.done():
            try:
                future.result()
                stats['success'] += 1
                stats['latencies'].append((time.time() - call_start) * 1000.0)
            except:
                stats['failed'] += 1
        else:
            stats['failed'] += 1

    lats = sorted(stats['latencies'])
    total = stats['success'] + stats['failed']
    p = lambda pct: lats[int(len(lats) * pct / 100.0)] if lats else 0
    
    print(f"\nOverall Statistics:\nTotal: {total}, Success: {stats['success']}, Failed: {stats['failed']}")
    print(f"Latency (ms): Avg: {sum(lats)/len(lats) if lats else 0:.2f}, Min: {lats[0] if lats else 0:.2f}, Max: {lats[-1] if lats else 0:.2f}, P50: {p(50):.2f}, P95: {p(95):.2f}, P99: {p(99):.2f}, P999: {p(99.9):.2f}")
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__': main()
