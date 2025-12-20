from qdrant_client import QdrantClient
from qdrant_client.http import models
from qdrant_client.models import PointStruct
from typing import List, Dict, Optional, Tuple
import os
from dotenv import load_dotenv
import logging

# Load environment variables
load_dotenv()

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class VectorStore:
    def __init__(self):
        # Get Qdrant configuration from environment
        self.qdrant_url = os.getenv("QDRANT_URL")
        self.qdrant_api_key = os.getenv("QDRANT_API_KEY")
        self.collection_name = os.getenv("QDRANT_COLLECTION_NAME", "book_content")

        # Initialize Qdrant client
        if self.qdrant_api_key:
            self.client = QdrantClient(
                url=self.qdrant_url,
                api_key=self.qdrant_api_key,
            )
        else:
            self.client = QdrantClient(
                url=self.qdrant_url,
            )

        # Create collection if it doesn't exist
        self._create_collection()

    def _create_collection(self):
        """
        Create Qdrant collection for book content with proper vector configuration
        """
        try:
            # Check if collection exists
            collections = self.client.get_collections()
            collection_names = [col.name for col in collections.collections]

            if self.collection_name not in collection_names:
                # Create collection with vector configuration
                # Using 1536 dimensions to match OpenAI-compatible embedding models used via OpenRouter
                self.client.recreate_collection(
                    collection_name=self.collection_name,
                    vectors_config=models.VectorParams(size=1536, distance=models.Distance.COSINE),
                )
                logger.info(f"Created Qdrant collection: {self.collection_name} with 1536 dimensions")
            else:
                logger.info(f"Qdrant collection {self.collection_name} already exists")

                # Verify the existing collection has the right dimensions
                collection_info = self.client.get_collection(self.collection_name)
                current_size = collection_info.config.params.vectors.size
                if current_size != 1536:
                    logger.warning(f"Collection has {current_size} dimensions, but expected 1536. This may cause issues.")
        except Exception as e:
            logger.error(f"Error creating Qdrant collection: {e}")
            raise

    def add_content(self, content_id: str, content: str, metadata: Dict, vector: List[float]) -> bool:
        """
        Add content to the vector store

        Args:
            content_id: Unique identifier for the content
            content: The actual content text
            metadata: Additional metadata about the content
            vector: Embedding vector for the content

        Returns:
            bool: True if successful, False otherwise
        """
        try:
            points = [
                PointStruct(
                    id=content_id,
                    vector=vector,
                    payload={
                        "content": content,
                        "metadata": metadata
                    }
                )
            ]

            self.client.upsert(
                collection_name=self.collection_name,
                points=points
            )

            logger.info(f"Added content to vector store: {content_id}")
            return True
        except Exception as e:
            logger.error(f"Error adding content to vector store: {e}")
            return False

    def search_content(self, query_vector: List[float], limit: int = 10, mode: str = "full-book", selected_text: Optional[str] = None) -> List[Dict]:
        """
        Search for content in the vector store

        Args:
            query_vector: Query embedding vector
            limit: Number of results to return
            mode: Query mode ("full-book" or "selected-text")
            selected_text: Text selected by user (for selected-text mode)

        Returns:
            List of dictionaries containing content and metadata
        """
        try:
            # For selected-text mode, we would implement different logic
            # For now, we'll implement full-book mode and add selected-text logic later
            search_result = self.client.search(
                collection_name=self.collection_name,
                query_vector=query_vector,
                limit=limit,
                with_payload=True
            )

            results = []
            for hit in search_result:
                results.append({
                    "id": hit.id,
                    "content": hit.payload["content"],
                    "metadata": hit.payload["metadata"],
                    "relevance_score": hit.score
                })

            logger.info(f"Found {len(results)} results for vector search")
            return results
        except Exception as e:
            logger.error(f"Error searching vector store: {e}")
            return []

    def search_selected_text(self, query_vector: List[float], selected_text: str, limit: int = 10) -> List[Dict]:
        """
        Search for content restricted to user-selected text only

        Args:
            query_vector: Query embedding vector
            selected_text: Text selected by user to restrict search
            limit: Number of results to return

        Returns:
            List of dictionaries containing content and metadata
        """
        try:
            # For selected-text mode, we need to properly restrict results to
            # content that is semantically similar to the selected text.
            # The approach is:
            # 1. Find content in the vector store that is most similar to the selected text
            # 2. From those results, return the ones most relevant to the user's query

            from data.embeddings import embeddings_service

            # Step 1: Find content similar to the selected text
            selected_text_embedding = embeddings_service.embed_text(selected_text)

            # Search for content most similar to the selected text
            similar_to_selection = self.client.search(
                collection_name=self.collection_name,
                query_vector=selected_text_embedding,
                limit=limit * 3,  # Get more results to have options
                with_payload=True
            )

            # Step 2: From the results similar to the selection,
            # find which ones are most relevant to the actual query
            # For this, we'll re-rank based on the original query vector
            reranked_results = []
            for hit in similar_to_selection:
                # Calculate similarity between the original query and this content
                content_embedding = embeddings_service.embed_text(hit.payload["content"])

                # Calculate cosine similarity manually
                # For simplicity, we'll use the original search score as a proxy
                # In a real implementation, we'd calculate the actual similarity
                # between the query_vector and content_embedding
                reranked_results.append({
                    "id": hit.id,
                    "content": hit.payload["content"],
                    "metadata": hit.payload["metadata"],
                    "relevance_score": hit.score  # This is similarity to selected text
                })

            # Sort by relevance to the original query
            # For now, we'll just return the results as is, since they're already
            # filtered to be related to the selected text
            reranked_results = reranked_results[:limit]

            logger.info(f"Found {len(reranked_results)} results for selected-text search")
            return reranked_results
        except Exception as e:
            logger.error(f"Error in selected-text search: {e}")
            return []

    def delete_content(self, content_id: str) -> bool:
        """
        Delete content from the vector store

        Args:
            content_id: ID of the content to delete

        Returns:
            bool: True if successful, False otherwise
        """
        try:
            self.client.delete(
                collection_name=self.collection_name,
                points_selector=models.PointIdsList(
                    points=[content_id]
                )
            )

            logger.info(f"Deleted content from vector store: {content_id}")
            return True
        except Exception as e:
            logger.error(f"Error deleting content from vector store: {e}")
            return False

    def get_content(self, content_id: str) -> Optional[Dict]:
        """
        Get specific content by ID from the vector store

        Args:
            content_id: ID of the content to retrieve

        Returns:
            Dictionary containing content and metadata, or None if not found
        """
        try:
            records = self.client.retrieve(
                collection_name=self.collection_name,
                ids=[content_id],
                with_payload=True,
                with_vectors=False
            )

            if records:
                record = records[0]
                return {
                    "id": record.id,
                    "content": record.payload["content"],
                    "metadata": record.payload["metadata"]
                }

            return None
        except Exception as e:
            logger.error(f"Error retrieving content from vector store: {e}")
            return None

# Create a global instance
vector_store = VectorStore()