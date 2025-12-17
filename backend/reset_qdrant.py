"""
Script to reset the Qdrant collection with correct dimensions
"""
from vector_store import vector_store
from qdrant_client.http import models

def reset_qdrant_collection():
    """Reset the Qdrant collection with correct dimensions"""
    print("Dropping existing Qdrant collection...")

    try:
        # Drop the existing collection
        vector_store.client.delete_collection(vector_store.collection_name)
        print(f"Deleted collection: {vector_store.collection_name}")
    except Exception as e:
        print(f"Collection may not have existed or error deleting: {e}")

    # Recreate collection with correct dimensions
    vector_store.client.recreate_collection(
        collection_name=vector_store.collection_name,
        vectors_config=models.VectorParams(size=1536, distance=models.Distance.COSINE),
    )
    print(f"Recreated collection: {vector_store.collection_name} with 1536 dimensions")

if __name__ == "__main__":
    reset_qdrant_collection()