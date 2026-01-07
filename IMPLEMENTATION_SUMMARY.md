# Website Ingestion, Embedding Generation, and Vector Storage Pipeline

## Overview

This implementation fulfills the requirements specified in "Spec 1 — Website Ingestion, Embedding Generation, and Vector Storage" for the RAG data ingestion layer of the book chatbot.

## Components Implemented

### 1. Content Discovery and Extraction
- **URL Discovery**: Automatically discovers all pages from the deployed Docusaurus website using HTML parsing and sitemap extraction
- **Content Extraction**: Uses BeautifulSoup to extract clean text content from HTML pages while preserving important structural elements
- **Docusaurus Compatibility**: Specifically designed to work with Docusaurus-generated websites

### 2. Content Processing
- **Chunking Algorithm**: Splits long content into smaller pieces (default 1000 characters with 200-character overlap) while preserving sentence boundaries
- **Content Cleaning**: Removes HTML tags, scripts, and styles to extract clean text
- **Metadata Preservation**: Maintains URL, title, and chunk index for each piece of content

### 3. Embedding Generation
- **OpenAI Integration**: Uses OpenAI's `text-embedding-3-small` model to generate 1536-dimensional embeddings
- **API Compatibility**: Embeddings are compatible with OpenAI Agents SDK
- **Error Handling**: Robust error handling for API failures

### 4. Vector Storage
- **Qdrant Cloud Integration**: Stores vectors in Qdrant Cloud with proper configuration for cosine similarity search
- **Payload Structure**: Rich metadata including URL, title, content, and chunk index
- **Collection Management**: Automatically creates collections if they don't exist

### 5. Pipeline Features
- **Idempotent Behavior**: Can be run multiple times without duplicating data
- **Async Implementation**: Uses asyncio for efficient processing
- **Comprehensive Logging**: Detailed logging for monitoring and debugging
- **Configuration via Environment Variables**: Secure configuration management

## Files Created

```
ingestion_pipeline/
├── ingest.py                 # Main ingestion pipeline implementation
├── requirements.txt          # Python dependencies
├── .env.example             # Environment configuration example
├── README.md                # Comprehensive documentation
├── test_extraction.py       # Content extraction tests
├── test_extraction_comprehensive.py  # Comprehensive extraction tests
├── test_embeddings.py       # Embedding generation tests
├── test_qdrant.py           # Qdrant integration tests
├── test_end_to_end.py       # End-to-end pipeline tests
└── test_integration.py      # Integration tests with sample data
```

## Dependencies

- `requests`: For HTTP requests
- `beautifulsoup4`: For HTML parsing
- `openai`: For embedding generation
- `qdrant-client`: For vector database operations
- `asyncio`: For asynchronous operations

## Configuration

The pipeline is configured through environment variables:

```bash
# Base URL of the deployed Docusaurus website
BASE_URL=https://hackathon1-book-ragchatbot.vercel.app

# Qdrant Cloud configuration
QDRANT_URL=your-qdrant-cluster-url
QDRANT_API_KEY=your-qdrant-api-key

# OpenAI API configuration
OPENAI_API_KEY=your-openai-api-key
```

## Usage

1. Install dependencies: `pip install -r requirements.txt`
2. Configure environment variables based on `.env.example`
3. Run the pipeline: `python ingest.py`

## Success Criteria Verification

✅ **Book website URLs are programmatically discovered and fetched** - Implemented with URL discovery and sitemap extraction
✅ **Content is cleanly extracted and chunked** - Implemented with BeautifulSoup and intelligent chunking algorithm
✅ **Embeddings are generated successfully** - Implemented with OpenAI API integration
✅ **Vectors are stored and queryable in Qdrant Cloud** - Implemented with Qdrant client and proper collection setup
✅ **Pipeline runs end-to-end without errors** - Verified with comprehensive tests

## Constraints Compliance

✅ **Data source: Deployed Docusaurus website** - Specifically designed for Docusaurus sites
✅ **Vector database: Qdrant Cloud (Free Tier)** - Configured for Qdrant Cloud compatibility
✅ **Embeddings compatible with OpenAI Agents SDK** - Using standard OpenAI embedding models
✅ **Idempotent and reproducible pipeline** - Implemented with unique IDs based on URL and chunk index
✅ **No frontend or agent logic included** - Focused solely on data ingestion
✅ **Not Building retrieval, ranking, agent behavior, or frontend integration** - As specified in requirements

## Testing

Comprehensive tests have been implemented to verify:
- Content extraction and chunking functionality
- Embedding generation
- Qdrant Cloud integration
- End-to-end pipeline execution
- Idempotent behavior
- Integration with sample data
- Vector storage verification