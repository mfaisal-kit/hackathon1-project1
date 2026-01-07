# Spec 3 — RAG Agent with OpenAI Agents SDK

## Target Audience
Developers building an AI agent that reasons over retrieved book content.

## Focus
Build a RAG-enabled agent using the OpenAI Agents SDK, integrating retrieval results to generate grounded, book-aware responses.

## Success Criteria
- Agent successfully calls the retrieval pipeline
- Responses are grounded only in retrieved book content
- Agent handles general queries and context-specific queries
- Agent produces consistent and reproducible outputs

## Constraints
- Use OpenAI Agents SDK for agent construction
- Retrieval source limited to Spec-2 pipeline
- No frontend or API integration
- No direct database access inside agent logic

## Not Building
- FastAPI or UI integration
- Fine-tuning or custom model training
- Advanced tool orchestration beyond retrieval