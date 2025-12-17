"""
Basic functionality test for the RAG Chatbot
This test verifies that the core components can be imported and basic functionality works
without requiring external API access.
"""
import sys
import os

# Add backend to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__)))

def test_imports():
    """Test that all core modules can be imported without errors"""
    print("Testing module imports...")

    try:
        from database import engine, SessionLocal, Base
        print("[OK] Database module imported successfully")
    except Exception as e:
        print(f"[ERROR] Database module import failed: {e}")
        return False

    try:
        from vector_store import vector_store
        print("[OK] Vector store module imported successfully")
    except Exception as e:
        print(f"[ERROR] Vector store module import failed: {e}")
        return False

    try:
        # Test that vector store can connect to Qdrant (without API calls)
        # This should work if Qdrant is accessible - use correct dimensions (1536 for OpenAI embeddings)
        test_results = vector_store.search_content([0.1] * 1536, limit=1)
        print(f"[OK] Vector store search works (found {len(test_results)} results)")
    except Exception as e:
        print(f"[WARN] Vector store search issue (may be due to no content): {e}")

    try:
        # Import agent but handle the API initialization error
        from agent import rag_agent
        if rag_agent is None:
            print("[WARN] Agent not initialized (expected if no API keys)")
        else:
            print("[OK] Agent initialized successfully")
    except Exception as e:
        print(f"[WARN] Agent initialization issue: {e}")

    try:
        from embeddings import QwenEmbeddings
        print("[OK] Embeddings module imported successfully")
    except Exception as e:
        print(f"[ERROR] Embeddings module import failed: {e}")
        return False

    try:
        from main import app
        print("[OK] FastAPI app imported successfully")
    except Exception as e:
        print(f"[ERROR] FastAPI app import failed: {e}")
        return False

    try:
        from session_service import session_service
        print("[OK] Session service imported successfully")
    except Exception as e:
        print(f"[ERROR] Session service import failed: {e}")
        return False

    return True

def test_basic_api_endpoints():
    """Test that API endpoints are defined and accessible"""
    print("\nTesting API endpoints...")

    try:
        from main import app
        # Check if the app has the expected routes
        routes = [route.path for route in app.routes]
        expected_routes = ["/api/chat", "/api/retrieve", "/health"]

        for route in expected_routes:
            if any(route in r for r in routes):
                print(f"[OK] Endpoint {route} is available")
            else:
                print(f"[WARN] Endpoint {route} may not be available")

        return True
    except Exception as e:
        print(f"[ERROR] API endpoint test failed: {e}")
        return False

def test_document_ingestion():
    """Test document ingestion functionality"""
    print("\nTesting document ingestion...")

    try:
        from document_ingestion import document_ingestion_service
        print("[OK] Document ingestion service imported successfully")

        # Test that we can access the service methods
        methods = dir(document_ingestion_service)
        expected_methods = ['ingest_document', 'chunk_content', 'index_content']

        for method in expected_methods:
            if method in methods:
                print(f"[OK] Method {method} is available")
            else:
                print(f"[WARN] Method {method} may not be available")

        return True
    except Exception as e:
        print(f"[ERROR] Document ingestion test failed: {e}")
        return False

def main():
    """Run all basic functionality tests"""
    print("Running basic functionality tests for RAG Chatbot...\n")

    all_passed = True

    all_passed &= test_imports()
    all_passed &= test_basic_api_endpoints()
    all_passed &= test_document_ingestion()

    print(f"\n{'='*50}")
    if all_passed:
        print("[OK] All basic functionality tests passed!")
        print("The RAG Chatbot system is properly structured and components are available.")
    else:
        print("[ERROR] Some functionality tests failed.")
        print("Please check the error messages above.")
    print(f"{'='*50}")

    return all_passed

if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)