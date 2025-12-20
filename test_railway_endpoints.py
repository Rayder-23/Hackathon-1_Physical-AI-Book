# Test script to verify Railway backend endpoints are functional
import requests
import json
import time

RAILWAY_URL = "https://[REDACTED].railway.app"

def test_health_endpoint():
    """Test the health check endpoint"""
    try:
        response = requests.get(f"{RAILWAY_URL}/health", timeout=10)
        print(f"Health check: {response.status_code}")
        if response.status_code == 200:
            data = response.json()
            print(f"Health response: {data}")
            return True
        else:
            print(f"Health check failed: {response.text}")
            return False
    except Exception as e:
        print(f"Health check error: {e}")
        return False

def test_root_endpoint():
    """Test the root endpoint"""
    try:
        response = requests.get(f"{RAILWAY_URL}/", timeout=10)
        print(f"Root endpoint: {response.status_code}")
        if response.status_code == 200:
            data = response.json()
            print(f"Root response keys: {list(data.keys())}")
            return True
        else:
            print(f"Root endpoint failed: {response.text}")
            return False
    except Exception as e:
        print(f"Root endpoint error: {e}")
        return False

def test_chat_endpoint():
    """Test the chat endpoint (without sending actual data to avoid errors)"""
    try:
        # This will likely return an error due to missing required fields, which is expected
        response = requests.post(f"{RAILWAY_URL}/api/chat", json={}, timeout=10)
        print(f"Chat endpoint: {response.status_code}")
        # 422 is expected for missing required fields
        if response.status_code in [422, 400, 401]:
            print("Chat endpoint reachable (expected error for missing data)")
            return True
        elif response.status_code == 200:
            print("Chat endpoint working")
            return True
        else:
            print(f"Chat endpoint unexpected response: {response.text}")
            return False
    except Exception as e:
        print(f"Chat endpoint error: {e}")
        return False

def main():
    print(f"Testing endpoints on Railway domain: {RAILWAY_URL}")
    print("="*50)

    print("\n1. Testing health endpoint...")
    health_ok = test_health_endpoint()

    print("\n2. Testing root endpoint...")
    root_ok = test_root_endpoint()

    print("\n3. Testing chat endpoint...")
    chat_ok = test_chat_endpoint()

    print("\n" + "="*50)
    print("Test Summary:")
    print(f"Health endpoint: {'[SUCCESS]' if health_ok else '[FAILED]'}")
    print(f"Root endpoint: {'[SUCCESS]' if root_ok else '[FAILED]'}")
    print(f"Chat endpoint: {'[SUCCESS]' if chat_ok else '[FAILED]'}")

    if health_ok and root_ok:
        print("\n[SUCCESS] Backend appears to be running and accessible!")
        print("You can now set REACT_APP_BACKEND_URL=https://[REDACTED].railway.app")
        print("in your frontend hosting environment variables.")
    else:
        print("\n[FAILED] Backend may not be accessible. Please check Railway deployment.")

if __name__ == "__main__":
    main()