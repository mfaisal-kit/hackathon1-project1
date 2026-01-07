# Website Ingestion Pipeline

This pipeline handles the extraction of content from the deployed Docusaurus website, generation of embeddings, and storage in Qdrant Cloud for the RAG chatbot.

## Overview

The ingestion pipeline performs the following steps:

1. Discovers all URLs from the deployed Docusaurus website
2. Extracts content from each URL
3. Chunks the content into manageable pieces
4. Generates embeddings using OpenAI's API
5. Stores the vectors in Qdrant Cloud for downstream retrieval

## Setup

### Prerequisites

- Python 3.8+
- Qdrant Cloud account (Free Tier)
- OpenAI API key

### Installation

1. Install the required dependencies:

```bash
pip install -r requirements.txt
```

2. Create a `.env` file based on the `.env.example`:

```bash
cp .env.example .env
```

3. Update the `.env` file with your actual configuration:

```env
# Base URL of the deployed Docusaurus website
BASE_URL=https://hackathon1-book-ragchatbot.vercel.app

# Qdrant Cloud configuration
QDRANT_URL=your-qdrant-cluster-url
QDRANT_API_KEY=your-qdrant-api-key

# OpenAI API configuration
OPENAI_API_KEY=your-openai-api-key
```

## Usage

To run the ingestion pipeline:

```bash
python ingest.py
```

## Configuration

The pipeline can be configured using environment variables:

- `BASE_URL`: The URL of the deployed Docusaurus website (default: `https://hackathon1-book-ragchatbot.vercel.app`)
- `QDRANT_URL`: Your Qdrant Cloud cluster URL
- `QDRANT_API_KEY`: Your Qdrant API key
- `OPENAI_API_KEY`: Your OpenAI API key

## Architecture

The pipeline is built with the following components:

- **URL Discovery**: Discovers all pages from the Docusaurus site using HTML parsing and sitemap extraction
- **Content Extraction**: Uses BeautifulSoup to extract clean text content from HTML pages
- **Content Chunking**: Splits long content into smaller chunks with overlap to preserve context
- **Embedding Generation**: Uses OpenAI's text-embedding-3-small model to generate vector embeddings
- **Vector Storage**: Stores vectors and metadata in Qdrant Cloud for efficient similarity search

## Idempotency

The pipeline is designed to be idempotent, meaning it can be run multiple times without duplicating data. Each chunk is assigned a unique ID based on its URL and position, preventing duplicates in Qdrant.

## Testing

To test the pipeline with sample data, you can run it against a local development server of the Docusaurus site by changing the `BASE_URL` to point to your local server (e.g., `http://localhost:3000`).