# Quickstart: AI Agent with Retrieval-Augmented Generation

## Overview
This guide will help you set up and run the AI agent with retrieval-augmented generation capabilities using the OpenAI Agent SDK and Qdrant integration.

## Prerequisites
- Python 3.9+
- OpenAI API key
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
pip install openai qdrant-client python-dotenv
```

### 3. Environment Configuration
Create a `.env` file in the backend directory with the following variables:

```env
OPENAI_API_KEY=your_openai_api_key_here
QDRANT_URL=your_qdrant_url_here
QDRANT_API_KEY=your_qdrant_api_key_here  # if required
QDRANT_COLLECTION_NAME=your_collection_name
```

### 4. Run the Agent
```bash
cd backend
python agent.py
```

## Usage Examples

### Initialize the Agent
```python
from agent import create_rag_agent

# Create the RAG agent with Qdrant integration
agent = create_rag_agent()
```

### Query the Agent
```python
# Ask a question about the book content
response = agent.query("What are the key principles of humanoid robotics?")
print(response)
```

### Advanced Usage
```python
# Get response with provenance information
response = agent.query_with_provenance("Explain the dynamics of robotic systems")
print(f"Answer: {response['answer']}")
print(f"Sources: {response['sources']}")
```

## Key Features

1. **Retrieval-Augmented Generation**: The agent retrieves relevant chunks from Qdrant and generates responses based only on this content
2. **Provenance Tracking**: All responses include citations to the specific book content used
3. **Fallback Handling**: When no relevant content is found, the agent responds appropriately
4. **Modular Design**: Easy to integrate into other projects

## Configuration Options

The agent can be configured with various parameters:

- `model`: OpenAI model to use (default: gpt-4-turbo)
- `temperature`: Response randomness (default: 0.1 for factual responses)
- `max_chunks`: Maximum number of chunks to retrieve (default: 5)
- `similarity_threshold`: Minimum similarity score for retrieved chunks (default: 0.7)

## Troubleshooting

### Common Issues

1. **API Key Issues**: Ensure your OpenAI and Qdrant API keys are correctly set in the environment
2. **Connection Issues**: Verify that your Qdrant instance is accessible
3. **No Results**: Check that your Qdrant collection contains properly indexed book content

### Performance Tips

- Adjust the `similarity_threshold` to balance between precision and recall
- Monitor response times and adjust the `max_chunks` parameter as needed
- Use appropriate OpenAI models based on your accuracy and cost requirements