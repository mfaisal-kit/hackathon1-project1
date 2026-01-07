# Spec 2 — Retrieval Pipeline and Validation

## Target Audience
Developers validating and testing the RAG retrieval layer for the book chatbot.

## Focus
Retrieve embedded book content from Qdrant and validate that the retrieval pipeline returns accurate, relevant, and grounded results.

## Success Criteria
- Queries successfully retrieve relevant vectors from Qdrant
- Retrieved chunks correctly map to source URLs and sections
- Retrieval results are consistent and reproducible
- End-to-end retrieval pipeline passes functional tests

## Constraints
- Vector store: Qdrant Cloud (Free Tier)
- Use embeddings generated in Spec-1
- Retrieval logic isolated from agent and frontend layers
- No new data ingestion or re-embedding

## Not Building
- Agent reasoning or response generation
- Frontend UI or API integration
- Reranking or advanced hybrid search