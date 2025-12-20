"""
Script to initialize the database with content from docs directory
"""
from database.database import create_tables
from data.document_ingestion import document_ingestion_service
from sqlalchemy.orm import sessionmaker
from database.database import engine

def initialize_database():
    """
    Initialize the database with content from docs directory
    """
    print("Creating database tables...")
    create_tables()
    print("Database tables created successfully!")

    print("Ingesting content from docs directory...")
    success = document_ingestion_service.ingest_docs_content()

    if success:
        print("Docs content ingested successfully!")
        print("The RAG system is now ready to answer questions about the Physical AI book content.")
    else:
        print("Error: Failed to ingest docs content")
        print("Falling back to sample content...")
        success = document_ingestion_service.ingest_sample_book()
        if success:
            print("Sample content ingested successfully as fallback.")
        else:
            print("Error: Both docs and sample content ingestion failed")

    return success

if __name__ == "__main__":
    initialize_database()