"""
Validation script for RAG functionality
"""
import asyncio
from agents.agent import rag_agent, QueryMode
from data.vector_store import vector_store
from data.document_ingestion import document_ingestion_service
import logging

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class RAGValidator:
    def __init__(self):
        self.test_questions = [
            {
                "question": "What is Physical AI?",
                "expected_context": ["Physical AI", "artificial intelligence", "physical world"],
                "mode": QueryMode.FULL_BOOK
            },
            {
                "question": "What are the fundamental principles of Physical AI?",
                "expected_context": ["perception", "reasoning", "action"],
                "mode": QueryMode.FULL_BOOK
            },
            {
                "question": "What challenges exist in Physical AI?",
                "expected_context": ["challenges", "sensor fusion", "uncertainty"],
                "mode": QueryMode.FULL_BOOK
            }
        ]

    def validate_retrieval_relevance(self):
        """
        Validate retrieval relevance and precision of vector search results
        """
        logger.info("Validating retrieval relevance...")

        total_tests = 0
        successful_retrievals = 0

        for test in self.test_questions:
            total_tests += 1

            # Perform retrieval
            from data.embeddings import embeddings_service
            query_embedding = embeddings_service.embed_text(test["question"])

            results = vector_store.search_content(
                query_vector=query_embedding,
                limit=5
            )

            # Check if expected context appears in results
            found_expected = False
            for result in results:
                content = result["content"].lower()
                for expected in test["expected_context"]:
                    if expected.lower() in content:
                        found_expected = True
                        break

            if found_expected:
                successful_retrievals += 1
                logger.info(f"✓ Retrieval successful for: {test['question'][:50]}...")
            else:
                logger.info(f"✗ Retrieval failed for: {test['question'][:50]}...")

        success_rate = successful_retrievals / total_tests if total_tests > 0 else 0
        logger.info(f"Retrieval relevance: {success_rate:.2%} ({successful_retrievals}/{total_tests})")

        return success_rate

    def validate_agent_responses(self):
        """
        Validate agent response quality and grounding in book content
        """
        logger.info("Validating agent responses...")

        total_tests = 0
        successful_responses = 0

        for test in self.test_questions:
            total_tests += 1

            # Process query through agent
            response = rag_agent.process_query(
                user_query=test["question"],
                mode=test["mode"]
            )

            # Check if response contains expected context
            response_content = response.content.lower()
            found_expected = any(expected.lower() in response_content
                               for expected in test["expected_context"])

            if found_expected and response.content and "no relevant content found" not in response.content.lower():
                successful_responses += 1
                logger.info(f"✓ Response successful for: {test['question'][:50]}...")
            else:
                logger.info(f"✗ Response failed for: {test['question'][:50]}...")

        success_rate = successful_responses / total_tests if total_tests > 0 else 0
        logger.info(f"Response quality: {success_rate:.2%} ({successful_responses}/{total_tests})")

        return success_rate

    def validate_selected_text_mode(self):
        """
        Validate selected-text mode enforcement
        """
        logger.info("Validating selected-text mode enforcement...")

        # Test with a specific text excerpt
        selected_text = "Physical AI represents a revolutionary approach to artificial intelligence that focuses on the interaction between AI systems and the physical world."

        test_question = "What does Physical AI focus on?"

        response = rag_agent.process_query(
            user_query=test_question,
            mode=QueryMode.SELECTED_TEXT,
            selected_text=selected_text
        )

        # Check if response is related to the selected text
        response_content = response.content.lower()
        is_related = any(word.lower() in response_content for word in ["physical", "ai", "world", "interaction"])

        if is_related and response.content and "no relevant content found" not in response.content.lower():
            logger.info("✓ Selected-text mode working correctly")
            return True
        else:
            logger.info("✗ Selected-text mode not working as expected")
            return False

    def validate_no_irrelevant_responses(self):
        """
        Validate that system returns appropriate response when no relevant content exists
        """
        logger.info("Validating response to irrelevant queries...")

        irrelevant_query = "What is the capital of France?"

        response = rag_agent.process_query(
            user_query=irrelevant_query,
            mode=QueryMode.FULL_BOOK
        )

        # Check if system properly rejects query unrelated to book content
        if "no relevant content found" in response.content.lower() or "not found" in response.content.lower():
            logger.info("✓ System properly rejects irrelevant queries")
            return True
        else:
            logger.info("✗ System should reject irrelevant queries")
            return False

    def run_all_validations(self):
        """
        Run all validation tests
        """
        logger.info("Starting RAG validation tests...")

        results = {
            "retrieval_relevance": self.validate_retrieval_relevance(),
            "response_quality": self.validate_agent_responses(),
            "selected_text_enforcement": self.validate_selected_text_mode(),
            "irrelevant_query_handling": self.validate_no_irrelevant_responses()
        }

        logger.info("Validation Summary:")
        logger.info(f"  Retrieval Relevance: {results['retrieval_relevance']:.2%}")
        logger.info(f"  Response Quality: {results['response_quality']:.2%}")
        logger.info(f"  Selected-text Enforcement: {'Pass' if results['selected_text_enforcement'] else 'Fail'}")
        logger.info(f"  Irrelevant Query Handling: {'Pass' if results['irrelevant_query_handling'] else 'Fail'}")

        return results

# Create a global instance
validator = RAGValidator()

if __name__ == "__main__":
    validator.run_all_validations()