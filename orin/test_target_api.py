#!/usr/bin/env python3
"""
Test script for Orin Target API

Tests the target coordinate API endpoints.
"""

import requests
import sys
import time

def test_api(base_url="http://localhost:8080"):
    """Test the Orin Target API"""
    
    print(f"Testing Orin Target API at {base_url}")
    print("=" * 60)
    
    # Test 1: Health check
    print("\n1. Testing health endpoint...")
    try:
        response = requests.get(f"{base_url}/health", timeout=5)
        print(f"   Status: {response.status_code}")
        print(f"   Response: {response.json()}")
        assert response.status_code == 200, "Health check failed"
        print("   ✓ Health check passed")
    except Exception as e:
        print(f"   ✗ Health check failed: {e}")
        return False
    
    # Test 2: Root endpoint
    print("\n2. Testing root endpoint...")
    try:
        response = requests.get(f"{base_url}/", timeout=5)
        print(f"   Status: {response.status_code}")
        print(f"   Response: {response.json()}")
        assert response.status_code == 200, "Root endpoint failed"
        print("   ✓ Root endpoint passed")
    except Exception as e:
        print(f"   ✗ Root endpoint failed: {e}")
        return False
    
    # Test 3: Send valid target coordinates
    print("\n3. Testing valid target coordinates...")
    test_coords = [
        (0.5, 0.5),   # Center
        (0.0, 0.0),   # Top-left
        (1.0, 1.0),   # Bottom-right
        (0.25, 0.75), # Random point
    ]
    
    for x, y in test_coords:
        try:
            response = requests.post(
                f"{base_url}/target",
                json={"x": x, "y": y},
                timeout=5
            )
            print(f"   Sending ({x}, {y}): {response.status_code}")
            if response.status_code == 200:
                data = response.json()
                print(f"      Response: {data}")
                assert data['status'] == 'success', "Unexpected status"
                assert data['x'] == x and data['y'] == y, "Coordinate mismatch"
                print(f"      ✓ Coordinates ({x}, {y}) sent successfully")
            else:
                print(f"      ✗ Failed: {response.text}")
                return False
        except Exception as e:
            print(f"      ✗ Error sending ({x}, {y}): {e}")
            return False
    
    # Test 4: Invalid coordinates (should fail)
    print("\n4. Testing invalid coordinates...")
    invalid_coords = [
        (1.5, 0.5),   # x out of range
        (0.5, -0.1),  # y out of range
        (2.0, 2.0),   # both out of range
    ]
    
    for x, y in invalid_coords:
        try:
            response = requests.post(
                f"{base_url}/target",
                json={"x": x, "y": y},
                timeout=5
            )
            if response.status_code != 200:
                print(f"   ✓ Correctly rejected invalid coordinates ({x}, {y})")
            else:
                print(f"   ✗ Should have rejected ({x}, {y})")
                return False
        except Exception as e:
            print(f"   Error testing invalid ({x}, {y}): {e}")
    
    # Test 5: Performance test
    print("\n5. Performance test (100 requests)...")
    start_time = time.time()
    success_count = 0
    
    for i in range(100):
        x = (i % 10) / 10.0
        y = (i % 7) / 10.0
        try:
            response = requests.post(
                f"{base_url}/target",
                json={"x": x, "y": y},
                timeout=5
            )
            if response.status_code == 200:
                success_count += 1
        except Exception:
            pass
    
    elapsed = time.time() - start_time
    print(f"   Completed {success_count}/100 requests in {elapsed:.2f}s")
    print(f"   Average: {elapsed/100*1000:.2f}ms per request")
    print(f"   Rate: {100/elapsed:.1f} req/s")
    
    if success_count >= 95:
        print("   ✓ Performance test passed")
    else:
        print(f"   ✗ Performance test failed (only {success_count}% success)")
        return False
    
    print("\n" + "=" * 60)
    print("All tests passed! ✓")
    return True


def main():
    """Main entry point"""
    base_url = sys.argv[1] if len(sys.argv) > 1 else "http://localhost:8080"
    
    print("Orin Target API Test Suite")
    print("=" * 60)
    print(f"Target URL: {base_url}")
    print()
    
    # Check if server is reachable
    print("Checking if server is running...")
    try:
        response = requests.get(f"{base_url}/health", timeout=2)
        print(f"✓ Server is reachable (status: {response.status_code})")
    except requests.exceptions.ConnectionError:
        print(f"✗ Cannot connect to {base_url}")
        print("  Make sure the target API server is running:")
        print("  python3 target_api.py")
        return 1
    except Exception as e:
        print(f"✗ Error connecting: {e}")
        return 1
    
    # Run tests
    success = test_api(base_url)
    
    return 0 if success else 1


if __name__ == "__main__":
    exit(main())
