from openai import OpenAI
import os
from dotenv import load_dotenv
import logging
from typing import Dict, List, Optional, Any
from pydantic import BaseModel, Field
from enum import Enum

# Import the vector store
from data.vector_store import vector_store

# Load environment variables
load_dotenv()

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class QueryMode(str, Enum):
    FULL_BOOK = "full-book"
    SELECTED_TEXT = "selected-text"

class AgentResponse(BaseModel):
    content: str
    citations: List[Dict] = Field(default_factory=list)
    token_usage: Dict[str, int] = Field(default_factory=dict)

class RetrievalTool:
    """
    Tool for retrieving relevant content from the Physical AI book
    """
    def __init__(self, vector_store):
        self.vector_store = vector_store
        self.name = "retrieve_book_content"
        self.description = "Retrieve relevant content from the Physical AI book"

    def get_definition(self):
        """
        Return the tool definition for OpenAI Agent
        """
        return {
            "type": "function",
            "function": {
                "name": self.name,
                "description": self.description,
                "parameters": {
                    "type": "object",
                    "properties": {
                        "query": {"type": "string", "description": "The search query"},
                        "mode": {"type": "string", "enum": ["full-book", "selected-text"], "description": "The query mode to use"},
                        "selected_text": {"type": "string", "description": "Text selected by user (required for selected-text mode)"}
                    },
                    "required": ["query", "mode"]
                }
            }
        }

    def __call__(self, query: str, mode: str, selected_text: Optional[str] = None) -> Dict:
        """
        Execute the retrieval tool

        Args:
            query: The search query
            mode: Query mode ("full-book" or "selected-text")
            selected_text: Text selected by user (for selected-text mode)

        Returns:
            Dictionary containing retrieved content and metadata
        """
        from data.embeddings import embeddings_service

        try:
            # Generate embedding for the query
            query_embedding = embeddings_service.embed_text(query)

            # Perform search based on mode
            if mode == "selected-text" and selected_text:
                # For selected-text mode, we search with restriction
                results = self.vector_store.search_selected_text(
                    query_vector=query_embedding,
                    selected_text=selected_text,
                    limit=5  # Limit results for relevance
                )
            else:
                # For full-book mode, search entire collection
                results = self.vector_store.search_content(
                    query_vector=query_embedding,
                    limit=5,
                    mode=mode
                )

            logger.info(f"Retrieved {len(results)} results for query: {query[:50]}...")

            return {
                "retrieved_content": results,
                "query": query,
                "mode": mode
            }
        except Exception as e:
            logger.error(f"Error in retrieval tool: {e}")
            return {
                "retrieved_content": [],
                "query": query,
                "mode": mode,
                "error": str(e)
            }

class ContextValidationTool:
    """
    Tool for validating that responses are grounded in book content
    """
    def __init__(self):
        self.name = "validate_context"
        self.description = "Validate that a response is grounded in book content"

    def get_definition(self):
        """
        Return the tool definition for OpenAI Agent
        """
        return {
            "type": "function",
            "function": {
                "name": self.name,
                "description": self.description,
                "parameters": {
                    "type": "object",
                    "properties": {
                        "response": {"type": "string", "description": "The response to validate"},
                        "retrieved_content": {"type": "array", "items": {"type": "string"}, "description": "Content that was retrieved"}
                    },
                    "required": ["response", "retrieved_content"]
                }
            }
        }

    def __call__(self, response: str, retrieved_content: List[str]) -> Dict:
        """
        Validate that the response is grounded in the retrieved content

        Args:
            response: The response to validate
            retrieved_content: Content that was retrieved for context

        Returns:
            Dictionary containing validation results
        """
        # Simple validation - check if response contains references to retrieved content
        # In a real implementation, this would be more sophisticated
        validation_score = 0.0
        issues = []

        if not retrieved_content:
            issues.append("No retrieved content to ground the response")
        else:
            # Check if response seems to reference the retrieved content
            response_lower = response.lower()
            content_found = False

            for content in retrieved_content:
                if len(content) > 20:  # Only check substantial content pieces
                    content_lower = content.lower()[:200]  # Check first 200 chars
                    if any(word in response_lower for word in content_lower.split()[:10]):
                        content_found = True
                        break

            if not content_found:
                issues.append("Response does not appear to reference retrieved content")
            else:
                validation_score = 0.8  # High confidence if content is referenced

        return {
            "is_valid": len(issues) == 0,
            "validation_score": validation_score,
            "issues": issues,
            "response": response
        }

class RAGAgent:
    """
    OpenAI Agent for RAG implementation with OpenRouter API compatibility
    """
    def __init__(self, vector_store, max_context_tokens: int = 2000):
        # Initialize OpenAI client with OpenRouter base URL
        # Use a more compatible initialization to avoid proxy issues
        import httpx
        # Create a custom httpx client without proxy settings that might cause issues
        http_client = httpx.Client(
            timeout=60.0,  # Set a reasonable timeout
        )

        # Get the base URL and ensure it's properly formatted for chat completions
        base_url = os.getenv("OPENROUTER_BASE_URL", "https://openrouter.ai/api/v1")
        # Make sure it doesn't have /chat/completions already appended
        base_url = base_url.rstrip("/chat/completions").rstrip("/")

        self.client = OpenAI(
            base_url=base_url,
            api_key=os.getenv("OPENROUTER_API_KEY"),
            http_client=http_client
        )

        self.vector_store = vector_store
        self.max_context_tokens = max_context_tokens
        # Use gpt-4o-mini as default for cost effectiveness
        self.model_name = os.getenv("OPENROUTER_MODEL_NAME", "openai/gpt-4o-mini")

        # If using the commented model format, extract the actual model name
        # (the .env has "anthropic/claude-3-sonnet # openai/gpt-4o-mini" format)
        if '#' in self.model_name:
            # Take the first part before the comment
            self.model_name = self.model_name.split('#')[0].strip()

        # Ensure we have a valid model name
        if not self.model_name:
            self.model_name = "openai/gpt-4o-mini"

        # Initialize tools
        self.retrieval_tool = RetrievalTool(vector_store)
        self.validation_tool = ContextValidationTool()

        # System prompt focused on Physical AI book content
        self.system_prompt = """You are an expert assistant for the Physical AI book. Your role is to answer questions based strictly on the content from the Physical AI book. You must:

1. Only provide information that is grounded in the retrieved book content
2. Cite specific sections, chapters, or pages when possible
3. If the question cannot be answered based on the book content, clearly state "No relevant content found in the book"
4. Do not hallucinate or provide information outside of the book content
5. Be helpful and provide clear explanations of complex topics

Always ensure your responses are factual and consistent with the book content."""

    def _format_retrieved_content(self, retrieved_items: List[Dict]) -> str:
        """
        Format retrieved content for inclusion in the prompt

        Args:
            retrieved_items: List of retrieved content items

        Returns:
            Formatted string of retrieved content
        """
        if not retrieved_items:
            return "No relevant content found in the book."

        formatted_content = []
        for item in retrieved_items:
            content = item.get("content", "")[:500]  # Limit content length
            metadata = item.get("metadata", {})
            chapter = metadata.get("chapter", "Unknown")
            section = metadata.get("section", "Unknown")
            page = metadata.get("page_number", "Unknown")

            formatted_item = f"""
Chapter: {chapter}
Section: {section}
Page: {page}
Content: {content}

Relevance Score: {item.get('relevance_score', 0.0):.2f}
---
"""
            formatted_content.append(formatted_item)

        # Combine and limit total length based on token count
        full_content = "\n".join(formatted_content)

        # Simple token estimation (1 token ~ 4 characters)
        estimated_tokens = len(full_content) // 4

        if estimated_tokens > self.max_context_tokens:
            # Truncate to fit token limit
            truncate_at = (self.max_context_tokens * 4) // len(formatted_content) * len(formatted_content) // 2
            full_content = full_content[:truncate_at]

        return full_content

    def _call_agent(self, user_query: str, mode: QueryMode, selected_text: Optional[str] = None, conversation_history: Optional[List[Dict[str, str]]] = None) -> AgentResponse:
        """
        Call the agent to process a user query

        Args:
            user_query: The user's question
            mode: Query mode (full-book or selected-text)
            selected_text: Text selected by user (for selected-text mode)
            conversation_history: List of previous messages in the conversation for context

        Returns:
            AgentResponse with content, citations, and token usage
        """
        try:
            # Step 1: Use retrieval tool to get relevant content
            retrieval_result = self.retrieval_tool(
                query=user_query,
                mode=mode.value,
                selected_text=selected_text
            )

            if retrieval_result.get("error"):
                logger.error(f"Retrieval tool error: {retrieval_result['error']}")
                return AgentResponse(
                    content="Error retrieving content from the book",
                    citations=[],
                    token_usage={}
                )

            retrieved_content = retrieval_result["retrieved_content"]

            # Format the retrieved content
            formatted_context = self._format_retrieved_content(retrieved_content)

            # If no content was found, return appropriate message
            if "No relevant content found" in formatted_context:
                return AgentResponse(
                    content="No relevant content found in the book",
                    citations=[],
                    token_usage={}
                )

            # Create the full prompt with conversation history if available
            if conversation_history:
                # Format conversation history
                history_text = "Previous conversation:\n"
                for msg in conversation_history[-5:]:  # Use last 5 messages to avoid token overflow
                    sender = msg.get("sender_type", "user")
                    content = msg.get("content", "")
                    history_text += f"{sender.capitalize()}: {content}\n"
                history_text += "\n"

                full_prompt = f"""
{history_text}
Question: {user_query}

Context from Physical AI book:
{formatted_context}

Based on the context above and the previous conversation, please answer the user's question. Remember to cite specific sections, chapters, or pages when possible, and only provide information that is grounded in the book content.
"""
            else:
                full_prompt = f"""
Question: {user_query}

Context from Physical AI book:
{formatted_context}

Based on the context above, please answer the user's question. Remember to cite specific sections, chapters, or pages when possible, and only provide information that is grounded in the book content.
"""

            # Call the OpenRouter API
            response = self.client.chat.completions.create(
                model=self.model_name,
                messages=[
                    {"role": "system", "content": self.system_prompt},
                    {"role": "user", "content": full_prompt}
                ],
                temperature=0.1,  # Lower temperature for more consistent, factual responses
                max_tokens=1000  # Limit response length
            )

            # Extract the response content
            content = response.choices[0].message.content

            # Extract token usage
            token_usage = {
                "prompt_tokens": response.usage.prompt_tokens,
                "completion_tokens": response.usage.completion_tokens,
                "total_tokens": response.usage.total_tokens
            }

            # Create citations from retrieved content
            citations = []
            for item in retrieved_content:
                metadata = item.get("metadata", {})
                citations.append({
                    "id": item.get("id"),
                    "chapter": metadata.get("chapter"),
                    "section": metadata.get("section"),
                    "page": metadata.get("page_number"),
                    "relevance_score": item.get("relevance_score", 0.0)
                })

            # Validate the response
            validation_result = self.validation_tool(
                response=content,
                retrieved_content=[item.get("content", "") for item in retrieved_content]
            )

            # If validation failed, adjust the response
            if not validation_result["is_valid"]:
                logger.warning(f"Response validation issues: {validation_result['issues']}")

            return AgentResponse(
                content=content,
                citations=citations,
                token_usage=token_usage
            )

        except Exception as e:
            logger.error(f"Error in agent call: {e}")
            return AgentResponse(
                content="Error processing your request",
                citations=[],
                token_usage={}
            )

    def process_query(self, user_query: str, mode: QueryMode = QueryMode.FULL_BOOK, selected_text: Optional[str] = None, conversation_history: Optional[List[Dict[str, str]]] = None) -> AgentResponse:
        """
        Process a user query through the RAG agent

        Args:
            user_query: The user's question
            mode: Query mode (full-book or selected-text)
            selected_text: Text selected by user (for selected-text mode)
            conversation_history: List of previous messages in the conversation for context

        Returns:
            AgentResponse with content, citations, and token usage
        """
        logger.info(f"Processing query in {mode.value} mode: {user_query[:50]}...")

        return self._call_agent(user_query, mode, selected_text, conversation_history)

# Create a global instance
rag_agent = RAGAgent(vector_store)