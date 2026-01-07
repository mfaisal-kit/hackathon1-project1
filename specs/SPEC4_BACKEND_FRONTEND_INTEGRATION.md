# Spec 4 — FastAPI Backend and Frontend Integration

## Target Audience
Developers integrating the RAG agent backend with the Docusaurus frontend.

## Focus
Expose the RAG agent through a FastAPI backend and connect it to the book's frontend to enable user queries and responses.

## Success Criteria
- FastAPI server runs locally and exposes a query endpoint
- Frontend successfully sends user queries to the backend
- Backend returns agent-generated, grounded responses
- End-to-end query flow works reliably

## Constraints
- Backend framework: FastAPI
- Agent logic reused from Spec-3
- Local frontend–backend communication only
- No production authentication or deployment

## Not Building
- Frontend UI redesign
- Cloud deployment or scaling
- Authentication or user management