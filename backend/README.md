# Book Content Ingestion Backend

This backend handles the ingestion of book content from the deployed Docusaurus website, generating embeddings and storing them in Qdrant Cloud for the RAG chatbot.

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
- UV package manager
- Qdrant Cloud account (Free Tier)
- OpenAI API key

### Installation

1. Install UV if you don't have it:
```bash
pip install uv
```

2. Navigate to the backend directory:
```bash
cd backend
```

3. Install dependencies using UV:
```bash
uv pip install -r requirements.txt
```

Or if using the pyproject.toml:
```bash
uv sync
```

4. Create a `.env` file with your configuration:

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
cd backend
uv run python main.py
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

## Main Components

The `main.py` file contains:

- `BookIngestor` class: Handles all ingestion logic including URL discovery, content extraction, chunking, embedding generation, and Qdrant storage
- `main()` function: Orchestrates the end-to-end ingestion pipeline

## Idempotency

The pipeline is designed to be idempotent, meaning it can be run multiple times without duplicating data. Each chunk is assigned a unique ID based on its URL and position, preventing duplicates in Qdrant.