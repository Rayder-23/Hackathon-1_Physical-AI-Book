import os
import uuid
from typing import List, Dict, Optional
from pathlib import Path
from database.database import SessionLocal
from models.models import BookContent
from data.vector_store import vector_store
from data.embeddings import embeddings_service
from dotenv import load_dotenv
import logging
import re

# Load environment variables
load_dotenv()

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class DocumentIngestionService:
    def __init__(self):
        self.chunk_size = 1000  # characters per chunk
        self.chunk_overlap = 100  # overlap between chunks

    def chunk_text(self, text: str, chunk_size: int = None, overlap: int = None) -> List[Dict]:
        """
        Split text into chunks with metadata preservation

        Args:
            text: Input text to chunk
            chunk_size: Size of each chunk in characters (default: self.chunk_size)
            overlap: Overlap between chunks (default: self.chunk_overlap)

        Returns:
            List of dictionaries containing chunk text and metadata
        """
        if chunk_size is None:
            chunk_size = self.chunk_size
        if overlap is None:
            overlap = self.chunk_overlap

        chunks = []
        start = 0

        while start < len(text):
            end = start + chunk_size

            # If we're near the end, make sure to include the rest
            if end > len(text):
                end = len(text)

            chunk_text = text[start:end]

            # Try to break at sentence boundaries to avoid cutting sentences
            if end < len(text):
                # Look for sentence endings near the end of the chunk
                sentence_end = max(
                    chunk_text.rfind('.'),
                    chunk_text.rfind('!'),
                    chunk_text.rfind('?'),
                    chunk_text.rfind('\n')
                )

                # If we found a sentence ending and it's not too far from the end
                if sentence_end > chunk_size * 0.7:  # At least 70% through the chunk
                    end = start + sentence_end + 1
                    chunk_text = text[start:end]

            chunk = {
                "text": chunk_text,
                "start_pos": start,
                "end_pos": end
            }

            chunks.append(chunk)

            # Move start position with overlap
            start = end - overlap

            # If the next chunk would be too small, just include the rest
            if len(text) - start < chunk_size // 2:
                if start < len(text):
                    final_chunk = {
                        "text": text[start:],
                        "start_pos": start,
                        "end_pos": len(text)
                    }
                    chunks.append(final_chunk)
                break

        logger.info(f"Split text into {len(chunks)} chunks")
        return chunks

    def process_book_content(self, content: str, section_title: str, chapter: str = None, section: str = None, page_number: int = None) -> List[Dict]:
        """
        Process book content into chunks with metadata

        Args:
            content: Full book content
            section_title: Title of the section
            chapter: Chapter name/number
            section: Section within the chapter
            page_number: Page reference

        Returns:
            List of processed content chunks with metadata
        """
        chunks = self.chunk_text(content)

        processed_chunks = []
        for i, chunk in enumerate(chunks):
            chunk_with_metadata = {
                "id": str(uuid.uuid4()),
                "section_title": section_title,
                "content": chunk["text"],
                "page_number": page_number,
                "chapter": chapter,
                "section": section,
                "chunk_index": i,
                "total_chunks": len(chunks)
            }
            processed_chunks.append(chunk_with_metadata)

        logger.info(f"Processed {len(processed_chunks)} chunks for section '{section_title}'")
        return processed_chunks

    def index_book_content(self, content_chunks: List[Dict]) -> bool:
        """
        Index book content chunks in both database and vector store

        Args:
            content_chunks: List of content chunks with metadata

        Returns:
            bool: True if successful, False otherwise
        """
        try:
            db = SessionLocal()

            for chunk_data in content_chunks:
                # Generate embedding for the content
                embedding = embeddings_service.embed_text(chunk_data["content"])

                # Save to database
                book_content = BookContent(
                    id=uuid.UUID(chunk_data["id"]),
                    section_title=chunk_data["section_title"],
                    content=chunk_data["content"],
                    content_embedding=str(embedding),  # Store as string representation
                    page_number=chunk_data["page_number"],
                    chapter=chunk_data["chapter"],
                    section=chunk_data["section"],
                )

                db.add(book_content)
                db.commit()

                # Save to vector store
                metadata = {
                    "section_title": chunk_data["section_title"],
                    "page_number": chunk_data["page_number"],
                    "chapter": chunk_data["chapter"],
                    "section": chunk_data["section"],
                    "chunk_index": chunk_data["chunk_index"],
                    "total_chunks": chunk_data["total_chunks"]
                }

                success = vector_store.add_content(
                    content_id=chunk_data["id"],
                    content=chunk_data["content"],
                    metadata=metadata,
                    vector=embedding
                )

                if not success:
                    logger.error(f"Failed to add content {chunk_data['id']} to vector store")
                    db.rollback()
                    return False

            db.close()
            logger.info(f"Successfully indexed {len(content_chunks)} content chunks")
            return True

        except Exception as e:
            logger.error(f"Error indexing book content: {e}")
            db.rollback()
            db.close()
            return False

    def validate_content_integrity(self, content_chunks: List[Dict]) -> Dict:
        """
        Validate the integrity of indexed content

        Args:
            content_chunks: List of content chunks to validate

        Returns:
            Dictionary with validation results
        """
        results = {
            "total_chunks": len(content_chunks),
            "valid_chunks": 0,
            "invalid_chunks": 0,
            "errors": []
        }

        for chunk_data in content_chunks:
            # Validate required fields
            if not chunk_data.get("id"):
                results["errors"].append(f"Chunk missing ID: {chunk_data.get('section_title', 'Unknown')}")
                results["invalid_chunks"] += 1
                continue

            if not chunk_data.get("content"):
                results["errors"].append(f"Chunk missing content: {chunk_data['id']}")
                results["invalid_chunks"] += 1
                continue

            if not chunk_data.get("section_title"):
                results["errors"].append(f"Chunk missing section title: {chunk_data['id']}")
                results["invalid_chunks"] += 1
                continue

            # Validate content length
            if len(chunk_data["content"]) < 10:  # Minimum 10 characters
                results["errors"].append(f"Chunk content too short: {chunk_data['id']}")
                results["invalid_chunks"] += 1
                continue

            results["valid_chunks"] += 1

        logger.info(f"Content validation complete: {results['valid_chunks']} valid, {results['invalid_chunks']} invalid")
        return results

    def load_markdown_files(self, docs_path: str = "../docs") -> List[Dict]:
        """
        Load all markdown files from the docs directory

        Args:
            docs_path: Path to the docs directory

        Returns:
            List of dictionaries containing file content, path, and metadata
        """
        import os
        from pathlib import Path

        markdown_files = []
        docs_dir = Path(docs_path)

        if not docs_dir.exists():
            logger.warning(f"Docs directory does not exist: {docs_path}")
            return []

        # Walk through all subdirectories and find markdown files
        for file_path in docs_dir.rglob("*.md"):
            try:
                with open(file_path, 'r', encoding='utf-8') as f:
                    content = f.read()

                # Extract relative path for metadata
                relative_path = file_path.relative_to(docs_dir)

                # Extract title from the first heading in the file
                title = self._extract_title_from_content(content)

                # Extract chapter/section info from path
                path_parts = str(relative_path).split('/')
                chapter = path_parts[0] if len(path_parts) > 1 else "Introduction"

                file_info = {
                    "content": content,
                    "title": title,
                    "path": str(relative_path),
                    "chapter": chapter,
                    "filename": file_path.name
                }

                markdown_files.append(file_info)
                logger.info(f"Loaded markdown file: {relative_path}")

            except Exception as e:
                logger.error(f"Error reading file {file_path}: {e}")

        return markdown_files

    def _extract_title_from_content(self, content: str) -> str:
        """
        Extract title from markdown content (first heading)

        Args:
            content: Markdown content

        Returns:
            Extracted title or a default title
        """
        import re
        # Look for the first heading in the content
        match = re.search(r'^#\s+(.+)$', content, re.MULTILINE)
        if match:
            return match.group(1).strip()
        # If no heading found, use filename as title
        return "Untitled Document"

    def load_sample_book_content(self) -> str:
        """
        Load sample book content for testing purposes

        Returns:
            Sample book content as a string
        """
        sample_content = """
        # Introduction to Physical AI

        Physical AI represents a revolutionary approach to artificial intelligence that focuses on
        the interaction between AI systems and the physical world. This field combines traditional
        AI techniques with robotics, computer vision, and sensor technologies to create systems
        that can perceive, reason about, and act in physical environments.

        The fundamental principles of Physical AI include perception, reasoning, and action.
        Perception involves sensing the environment through various modalities such as vision,
        touch, and sound. Reasoning encompasses the cognitive processes that interpret sensory
        data and make decisions. Action refers to the physical manifestation of AI decisions
        through robotic systems or other physical interfaces.

        One of the key challenges in Physical AI is the integration of multiple sensory inputs
        to form a coherent understanding of the environment. This requires sophisticated
        sensor fusion techniques and robust algorithms that can handle uncertainty and noise
        in sensor data.

        Another significant challenge is the real-time processing requirements of physical
        systems. Unlike traditional AI systems that can take time to process information,
        Physical AI systems must respond quickly to dynamic changes in their environment.

        The applications of Physical AI are vast and include autonomous vehicles, robotic
        assistants, smart manufacturing, and augmented reality systems. These applications
        require AI systems that can operate safely and effectively in complex physical
        environments.

        As the field continues to evolve, new challenges emerge in areas such as ethical
        considerations, safety standards, and human-AI collaboration. The future of Physical
        AI will likely involve increasingly sophisticated systems that can seamlessly
        integrate with human activities and enhance our interaction with the physical world.
        """
        return sample_content

    def ingest_docs_content(self, docs_path: str = "../docs") -> bool:
        """
        Ingest content from markdown files in the docs directory

        Args:
            docs_path: Path to the docs directory

        Returns:
            bool: True if successful, False otherwise
        """
        try:
            logger.info(f"Starting docs content ingestion from: {docs_path}")

            # Load all markdown files
            markdown_files = self.load_markdown_files(docs_path)

            if not markdown_files:
                logger.warning("No markdown files found in docs directory")
                # Fallback to sample content if no files found
                logger.info("Using sample content as fallback")
                return self.ingest_sample_book()

            all_content_chunks = []

            # Process each markdown file
            for file_info in markdown_files:
                logger.info(f"Processing file: {file_info['path']}")

                # Process content into chunks
                file_chunks = self.process_book_content(
                    content=file_info["content"],
                    section_title=file_info["title"],
                    chapter=file_info["chapter"],
                    section=file_info["filename"].replace('.md', ''),
                    page_number=None  # Not applicable for markdown docs
                )

                all_content_chunks.extend(file_chunks)

            if not all_content_chunks:
                logger.error("No content chunks were created from docs")
                return False

            # Validate content integrity
            validation_results = self.validate_content_integrity(all_content_chunks)

            if validation_results["invalid_chunks"] > 0:
                logger.error(f"Content validation failed with {validation_results['invalid_chunks']} invalid chunks")
                return False

            # Index the content
            success = self.index_book_content(all_content_chunks)

            if success:
                logger.info(f"Docs content ingestion completed successfully. Indexed {len(all_content_chunks)} chunks from {len(markdown_files)} files.")
            else:
                logger.error("Docs content ingestion failed")

            return success

        except Exception as e:
            logger.error(f"Error in docs content ingestion: {e}")
            return False

    def ingest_sample_book(self) -> bool:
        """
        Ingest sample book content for testing

        Returns:
            bool: True if successful, False otherwise
        """
        try:
            logger.info("Starting sample book ingestion process")

            # Load sample content
            sample_content = self.load_sample_book_content()

            # Process content into chunks
            content_chunks = self.process_book_content(
                content=sample_content,
                section_title="Introduction to Physical AI",
                chapter="Chapter 1",
                section="Introduction",
                page_number=1
            )

            # Validate content integrity
            validation_results = self.validate_content_integrity(content_chunks)

            if validation_results["invalid_chunks"] > 0:
                logger.error(f"Content validation failed with {validation_results['invalid_chunks']} invalid chunks")
                return False

            # Index the content
            success = self.index_book_content(content_chunks)

            if success:
                logger.info("Sample book ingestion completed successfully")
            else:
                logger.error("Sample book ingestion failed")

            return success

        except Exception as e:
            logger.error(f"Error in sample book ingestion: {e}")
            return False

# Create a global instance
document_ingestion_service = DocumentIngestionService()