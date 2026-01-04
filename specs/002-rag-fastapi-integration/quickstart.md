# Quickstart: RAG FastAPI Integration

## Overview
This guide will help you set up and run the FastAPI server that integrates with the RAG agent for frontend-backend communication.

## Prerequisites
- Python 3.9+
- OpenAI API key (or OpenRouter API key if using OpenRouter)
- Qdrant instance with book content indexed
- Git

## Setup

### 1. Clone the Repository
```bash
git clone <repository-url>
cd <repository-name>
```

### 2. Install Dependencies
```bash
cd backend
pip install -r requirements.txt
```

If no requirements.txt exists yet, install the required packages:
```bash
pip install fastapi uvicorn python-dotenv openai qdrant-client
```

### 3. Environment Configuration
Create a `.env` file in the backend directory with the following variables:

```env
OPENAI_API_KEY=your_openai_api_key_here
# OR if using OpenRouter:
OPENROUTER_API_KEY=your_openrouter_api_key_here
QDRANT_URL=your_qdrant_url_here
QDRANT_API_KEY=your_qdrant_api_key_here  # if required
QDRANT_COLLECTION_NAME=your_collection_name
```

### 4. Run the FastAPI Server
```bash
cd backend
uvicorn api:app --reload --host 0.0.0.0 --port 8000
```

## API Usage

### Query Endpoint
**Endpoint**: `POST /api/query`

**Request**:
```json
{
  "query": "What are the key principles of humanoid robotics?",
  "top_k": 5,
  "temperature": 0.1
}
```

**Response**:
```json
{
  "answer": "The key principles of humanoid robotics include...",
  "sources": [
    {
      "title": "Introduction to Humanoid Robotics",
      "url": "https://example.com/chapter1",
      "similarity_score": 0.85,
      "chunk_id": "chunk_123",
      "document_id": "doc_456"
    }
  ],
  "success": true,
  "performance": {
    "total_duration": 2.45,
    "retrieve_duration": 1.2,
    "generate_duration": 1.25
  }
}
```

## Frontend Integration

The API is designed to work with the existing Docusaurus-based frontend in the `physicalAI-book/` directory. The frontend can make POST requests to the `/api/query` endpoint and display the responses to users.

### Example Frontend Request
```javascript
fetch('http://localhost:8000/api/query', {
  method: 'POST',
  headers: {
    'Content-Type': 'application/json',
  },
  body: JSON.stringify({
    query: 'Explain cognitive planning concepts',
    top_k: 3,
    temperature: 0.1
  })
})
.then(response => response.json())
.then(data => {
  console.log('Response:', data);
  // Display the answer and sources in the UI
});
```

## Key Features

1. **FastAPI Server**: Provides automatic API documentation at `/docs`
2. **JSON Communication**: Standard JSON request/response format
3. **RAG Integration**: Calls the existing RAG agent for query processing
4. **Error Handling**: Graceful error handling with appropriate HTTP status codes
5. **Performance Metrics**: Includes timing information in responses
6. **Concurrent Requests**: Handles multiple requests simultaneously

## Configuration Options

The API can be configured with various parameters:

- `top_k`: Number of results to retrieve from knowledge base (default: 5)
- `temperature`: Response randomness (default: 0.1 for factual responses)

## Troubleshooting

### Common Issues

1. **API Key Issues**: Ensure your OpenAI or OpenRouter API keys are correctly set in the environment
2. **Connection Issues**: Verify that your Qdrant instance is accessible
3. **CORS Issues**: If making requests from a browser, ensure CORS is configured properly
4. **No Results**: Check that your Qdrant collection contains properly indexed book content

### Performance Tips

- Monitor response times and adjust the `top_k` parameter as needed
- Use appropriate OpenAI models based on your accuracy and cost requirements
- Consider implementing caching for frequently asked questions