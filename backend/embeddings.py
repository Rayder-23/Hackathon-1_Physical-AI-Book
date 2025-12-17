import requests
import os
from dotenv import load_dotenv
import logging
from typing import List, Union

# Load environment variables
load_dotenv()

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class QwenEmbeddings:
    def __init__(self):
        # Check if OpenRouter is working first (since we know the OpenRouter API works from agent initialization)
        openrouter_key = os.getenv("OPENROUTER_API_KEY")
        if openrouter_key:
            # Use OpenRouter for embeddings since it's known to work
            self.api_key = openrouter_key
            self.base_url = os.getenv("OPENROUTER_BASE_URL", "https://openrouter.ai/api/v1").replace("/chat/completions", "")
            self.embedding_model = os.getenv("OPENROUTER_EMBEDDING_MODEL", "text-embedding-3-small")  # Use a proper embedding model
        else:
            # Fallback to Qwen if OpenRouter is not available
            self.api_key = os.getenv("QWEN_EMBEDDING_API_KEY")
            self.base_url = os.getenv("QWEN_EMBEDDING_BASE_URL", "https://dashscope-intl.aliyuncs.com/compatible-mode/v1")
            self.embedding_model = "text-embedding-v1"

        if not self.api_key:
            raise ValueError("Either OPENROUTER_API_KEY or QWEN_EMBEDDING_API_KEY environment variable is required")

        # Ensure base URL ends correctly
        if "openrouter" in self.base_url.lower() and not self.base_url.endswith("/v1"):
            self.base_url = self.base_url.rstrip("/chat/completions")
            if not self.base_url.endswith("/v1"):
                self.base_url += "/v1"

    def _make_request(self, text: Union[str, List[str]]) -> dict:
        """
        Make request to Qwen API for embeddings
        """
        # Handle both single string and list of strings
        if isinstance(text, str):
            input_text = [text]
        else:
            input_text = text

        # Try DashScope's direct API endpoint first (this is the correct Qwen/DashScope format)
        # DashScope may require different authentication format
        dashscope_headers = {
            "Authorization": f"Bearer {self.api_key}",
            "Content-Type": "application/json"
        }

        payload = {
            "model": self.embedding_model,
            "input": {"texts": input_text if isinstance(input_text, list) else [input_text]}
        }

        # Try the DashScope direct endpoint first
        dashscope_url = "https://dashscope.aliyuncs.com/api/v1/services/embed/text-embedding"
        try:
            response = requests.post(
                dashscope_url,
                headers=dashscope_headers,
                json=payload
            )
            response.raise_for_status()
            return response.json()
        except requests.exceptions.RequestException as e:
            logger.error(f"Error calling DashScope direct embeddings API: {e}")

            # Fallback to OpenRouter which should work with standard OpenAI format
            openrouter_headers = {
                "Authorization": f"Bearer {self.api_key}",
                "Content-Type": "application/json"
            }

            try:
                payload_openrouter = {
                    "model": self.embedding_model,
                    "input": input_text
                }
                response = requests.post(
                    f"{self.base_url}/embeddings",
                    headers=openrouter_headers,
                    json=payload_openrouter
                )
                response.raise_for_status()
                return response.json()
            except requests.exceptions.RequestException as alt_e:
                logger.error(f"Error calling fallback embeddings API: {alt_e}")

                # Final fallback: try with Qwen/DashScope specific format and authentication
                try:
                    # Some DashScope implementations require the key in a different header
                    qwen_headers = {
                        "Authorization": f"Bearer {self.api_key}",
                        "X-DashScope-Async": "enable",  # Sometimes required
                        "Content-Type": "application/json"
                    }
                    response = requests.post(
                        f"{self.base_url}/embeddings",  # Try original URL with Qwen format
                        headers=qwen_headers,
                        json={
                            "model": self.embedding_model,
                            "input": {"texts": input_text if isinstance(input_text, list) else [input_text]}
                        }
                    )
                    response.raise_for_status()
                    return response.json()
                except requests.exceptions.RequestException as final_e:
                    logger.error(f"Error with final fallback: {final_e}")
                    raise

    def embed_text(self, text: str) -> List[float]:
        """
        Generate embedding for a single text

        Args:
            text: Input text to embed

        Returns:
            List of floats representing the embedding vector
        """
        try:
            result = self._make_request(text)
            embedding = result["data"][0]["embedding"]
            logger.info(f"Generated embedding for text of length {len(text)}")
            return embedding
        except Exception as e:
            logger.error(f"Error generating embedding: {e}")
            raise

    def embed_texts(self, texts: List[str]) -> List[List[float]]:
        """
        Generate embeddings for a list of texts

        Args:
            texts: List of input texts to embed

        Returns:
            List of embedding vectors (each vector is a list of floats)
        """
        try:
            result = self._make_request(texts)
            embeddings = [item["embedding"] for item in result["data"]]
            logger.info(f"Generated embeddings for {len(texts)} texts")
            return embeddings
        except Exception as e:
            logger.error(f"Error generating embeddings: {e}")
            raise

    def get_embedding_dimensions(self) -> int:
        """
        Get the dimensionality of the embeddings

        Returns:
            Number of dimensions in the embedding vector
        """
        # Use a short test text to determine dimensions
        try:
            test_embedding = self.embed_text("test")
            return len(test_embedding)
        except Exception as e:
            logger.error(f"Error getting embedding dimensions: {e}")
            return 768  # Default fallback

# Create a global instance
embeddings_service = QwenEmbeddings()