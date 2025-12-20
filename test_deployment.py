# Test script to verify the backend can start without errors
import sys
import os

# Add the backend directory to Python path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'backend'))

try:
    # Try to import the main app to check for basic import errors
    from backend.main import app
    print("[SUCCESS] Backend imports successfully")
    print("[SUCCESS] FastAPI app initialized")

    # Check if required environment variables are available (or have defaults)
    import os
    required_vars = [
        'OPENROUTER_API_KEY',
        'QDRANT_URL',
        'QDRANT_API_KEY',
        'DATABASE_URL'
    ]

    missing_vars = []
    for var in required_vars:
        if not os.getenv(var):
            missing_vars.append(var)

    if missing_vars:
        print(f"[WARNING] Missing environment variables: {', '.join(missing_vars)}")
        print("  (This is expected in local testing, but required for production)")
    else:
        print("[SUCCESS] All required environment variables are set")

    print("\n[SUCCESS] Deployment configuration appears to be set up correctly!")
    print("\nTo deploy to Railway:")
    print("1. Install the Railway CLI: `npm install -g @railway/cli`")
    print("2. Login: `railway login`")
    print("3. Link your project: `railway link`")
    print("4. Set environment variables in Railway dashboard or CLI")
    print("5. Deploy: `railway up` or `railway deploy`")

except ImportError as e:
    print(f"[ERROR] Import error: {e}")
    sys.exit(1)
except Exception as e:
    print(f"[ERROR] Error: {e}")
    sys.exit(1)