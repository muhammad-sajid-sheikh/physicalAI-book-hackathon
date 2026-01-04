#!/usr/bin/env python3
"""
FastAPI Server for RAG Agent Integration

This module implements a FastAPI server that exposes a query endpoint to integrate
with the RAG agent for frontend-backend communication. The API accepts JSON requests
with user queries, processes them through the RAG agent, and returns JSON responses.
"""

from fastapi import FastAPI, HTTPException, status
from pydantic import BaseModel, Field
from typing import List, Optional, Dict, Any
import logging
import os
import time
from dotenv import load_dotenv

# Import the RAG agent
from agent import RAGAgent

# Load environment variables
load_dotenv()

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

# Create global RAGAgent instance for reuse
try:
    rag_agent = RAGAgent()
    logger.info("RAG Agent initialized successfully")
except Exception as e:
    logger.error(f"Failed to initialize RAG Agent: {str(e)}")
    rag_agent = None

# Create FastAPI app instance
app = FastAPI(
    title="RAG Agent API",
    description="API for RAG agent integration with frontend",
    version="1.0.0"
)

# Pydantic models for request/response validation
class Source(BaseModel):
    """Information about a source used to generate the response"""
    title: str = Field(..., description="Title of the source document")
    url: str = Field(..., description="URL of the source")
    similarity_score: float = Field(..., description="Relevance score of the source", ge=0.0, le=1.0)
    chunk_id: str = Field(..., description="ID of the specific content chunk")
    document_id: str = Field(..., description="ID of the source document")


class PerformanceMetrics(BaseModel):
    """Performance metrics for the query processing"""
    total_duration: float = Field(..., description="Total time to process the request in seconds")
    retrieve_duration: float = Field(..., description="Time spent retrieving content in seconds")
    generate_duration: float = Field(..., description="Time spent generating response in seconds")


class QueryRequest(BaseModel):
    """Request model for the query endpoint"""
    query: str = Field(..., description="The user's question or query text", min_length=1)
    top_k: Optional[int] = Field(5, description="Number of results to retrieve from knowledge base", ge=1, le=10)
    temperature: Optional[float] = Field(0.1, description="Response randomness parameter", ge=0.0, le=1.0)


class QueryResponse(BaseModel):
    """Response model for the query endpoint"""
    answer: str = Field(..., description="The agent's response to the query")
    sources: List[Source] = Field(default_factory=list, description="List of sources used to generate the response")
    success: bool = Field(True, description="Whether the query was processed successfully")
    error: Optional[str] = Field(None, description="Error message if processing failed")
    performance: Optional[PerformanceMetrics] = Field(None, description="Performance metrics for the request")


@app.get("/")
async def root():
    """
    Root endpoint to verify the API is running

    Returns:
        dict: Basic API status information
    """
    return {"message": "RAG Agent API is running", "status": "healthy"}


@app.get("/health")
async def health_check():
    """Health check endpoint"""
    # TODO: Implement actual health checks for RAG agent, OpenAI, and Qdrant connectivity
    return {
        "status": "healthy",
        "timestamp": time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime()),
        "services": {
            "openai": True,  # Placeholder - will be updated after RAG agent integration
            "qdrant": True,  # Placeholder - will be updated after RAG agent integration
            "rag_agent": True  # Placeholder - will be updated after RAG agent integration
        }
    }


@app.post("/api/query", response_model=QueryResponse, status_code=status.HTTP_200_OK)
async def query_endpoint(request: QueryRequest):
    """
    Query endpoint that processes user queries through the RAG agent.
    """
    logger.info(f"Received query: {request.query[:50]}{'...' if len(request.query) > 50 else ''}")

    # Check if RAG agent is available
    if rag_agent is None:
        error_msg = "RAG Agent is not available"
        logger.error(error_msg)
        return QueryResponse(
            answer="",
            sources=[],
            success=False,
            error=error_msg
        )

    try:
        # Process the query using the RAG agent
        start_time = time.time()

        # Call the RAG agent to process the query with timeout protection
        import concurrent.futures
        import threading

        # Use ThreadPoolExecutor to run the RAG agent query with timeout
        with concurrent.futures.ThreadPoolExecutor() as executor:
            # Submit the query to the thread pool
            future = executor.submit(
                rag_agent.query_with_provenance,
                user_query=request.query,
                top_k=request.top_k
            )

            try:
                # Wait for the result with a timeout of 30 seconds
                result = future.result(timeout=30.0)
            except concurrent.futures.TimeoutError:
                error_msg = "Request timed out processing query with RAG agent"
                logger.error(error_msg)
                return QueryResponse(
                    answer="",
                    sources=[],
                    success=False,
                    error=error_msg
                )

        total_duration = time.time() - start_time

        # Create performance metrics
        perf_metrics = PerformanceMetrics(
            total_duration=round(total_duration, 3),
            retrieve_duration=result.get('performance', {}).get('retrieve_duration', 0),
            generate_duration=result.get('performance', {}).get('generate_duration', 0)
        )

        # Create source objects from the result
        sources = []
        for source_data in result.get('sources', []):
            source = Source(
                title=source_data.get('title', 'Unknown'),
                url=source_data.get('url', ''),
                similarity_score=source_data.get('similarity_score', 0.0),
                chunk_id=source_data.get('chunk_id', ''),
                document_id=source_data.get('document_id', '')
            )
            sources.append(source)

        # Check if the response indicates no relevant content was found
        answer_text = result.get('answer', '')
        no_content_found = "I don't have information about this in the available book content." in answer_text

        response = QueryResponse(
            answer=answer_text,
            sources=sources,
            success=not no_content_found,  # Mark as unsuccessful if no content found
            performance=perf_metrics
        )

        logger.info(f"Query processed successfully in {total_duration:.3f}s")
        return response

    except HTTPException:
        # Re-raise HTTP exceptions
        raise
    except Exception as e:
        error_msg = f"Error processing query: {str(e)}"
        logger.error(error_msg)
        return QueryResponse(
            answer="",
            sources=[],
            success=False,
            error=error_msg
        )


# Add exception handlers for different error scenarios
@app.exception_handler(400)
async def validation_exception_handler(request, exc):
    """Handle validation errors with appropriate status codes"""
    return QueryResponse(
        answer="",
        sources=[],
        success=False,
        error=f"Validation error: {str(exc)}"
    )


if __name__ == "__main__":
    import uvicorn

    # Run the server
    uvicorn.run(
        "api:app",
        host="0.0.0.0",
        port=int(os.getenv("PORT", 8000)),
        reload=True,
        log_level="info"
    )