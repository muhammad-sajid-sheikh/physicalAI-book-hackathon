#!/usr/bin/env python3
"""
AI Agent with Retrieval-Augmented Generation

This module implements an AI agent using the OpenAI Agents SDK that integrates
with Qdrant for retrieval-augmented generation. The agent retrieves relevant
book content from Qdrant and generates responses based only on the retrieved chunks.
"""

import os
import json
import logging
from typing import Dict, List, Any, Optional
from urllib.error import URLError
from requests.exceptions import ConnectionError
from dotenv import load_dotenv
from openai import OpenAI
from openai.types.beta.assistant import Assistant
from openai.types.beta.thread import Thread
from openai.types.beta.threads.run import Run
from retrieval import QdrantRetriever


# Load environment variables
load_dotenv()

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

class RAGAgent:
    """
    AI Agent with Retrieval-Augmented Generation capabilities.

    This agent uses the OpenAI Agent SDK and integrates with Qdrant
    to retrieve relevant book content and generate responses based only
    on the retrieved information.
    """

    def __init__(self, model: str = "gpt-4-turbo", temperature: float = 0.1):
        """
        Initialize the RAG Agent.

        Args:
            model: OpenAI model to use for generation
            temperature: Temperature for response generation (lower for more factual responses)
        """
        try:
            # Initialize OpenRouter client
            openrouter_api_key = os.getenv('OPENROUTER_API_KEY')
            if not openrouter_api_key:
                raise ValueError("OPENROUTER_API_KEY environment variable not set")

            self.client = OpenAI(
                api_key=openrouter_api_key,
                base_url="https://openrouter.ai/api/v1"
            )
            self.model = model
            self.temperature = temperature

            # Initialize Qdrant retriever
            self.qdrant_retriever = QdrantRetriever()

            # Create an assistant with the retrieval tool
            self.assistant = self.client.beta.assistants.create(
                name="RAG Book Content Assistant",
                instructions="""You are an AI assistant that answers questions based only on the provided retrieved contexts.
                Do not use any prior knowledge or information outside of what is provided in the retrieved context.
                If the information is not present in the context, clearly state that you don't have that information.
                Always provide answers that are factual and based solely on the retrieved content.""",
                model=self.model,
                tools=[
                    {
                        "type": "function",
                        "function": {
                            "name": "retrieve_book_content",
                            "description": "Retrieve relevant book content from Qdrant based on a query",
                            "parameters": {
                                "type": "object",
                                "properties": {
                                    "query": {
                                        "type": "string",
                                        "description": "The search query to find relevant book content"
                                    },
                                    "top_k": {
                                        "type": "integer",
                                        "description": "Number of results to retrieve (default: 5)",
                                        "default": 5
                                    }
                                },
                                "required": ["query"]
                            }
                        }
                    }
                ]
            )

            logger.info("RAG Agent initialized successfully with OpenRouter Assistant")
        except Exception as e:
            logger.error(f"Error initializing RAG Agent: {str(e)}")
            raise

    def retrieve_content(self, query: str, top_k: int = 5) -> List[Dict[str, Any]]:
        """
        Retrieve relevant content from Qdrant based on the query.

        Args:
            query: User query to search for
            top_k: Number of results to retrieve

        Returns:
            List of retrieved content chunks with metadata
        """
        try:
            logger.info(f"Retrieving content for query: '{query}'")
            results = self.qdrant_retriever.retrieve_content(query, top_k=top_k)
            logger.info(f"Retrieved {len(results)} chunks from Qdrant")
            return results
        except ConnectionError as e:
            logger.error(f"Qdrant service unavailable: {str(e)}")
            raise
        except Exception as e:
            logger.error(f"Error retrieving content from Qdrant: {str(e)}")
            raise

    def retrieve_book_content(self, query: str, top_k: int = 5):
        """
        Retrieve book content from Qdrant. This method is designed to be used as an OpenAI tool.

        Args:
            query: The search query to find relevant book content
            top_k: Number of results to retrieve (default: 5)

        Returns:
            JSON string of retrieved content
        """
        try:
            results = self.retrieve_content(query, top_k=top_k)
            return json.dumps(results, default=str)  # Convert to JSON string for OpenAI
        except Exception as e:
            logger.error(f"Error in retrieve_book_content tool: {str(e)}")
            return json.dumps({"error": str(e)})

    def query(self, user_query: str, top_k: int = 5) -> str:
        """
        Process a user query using retrieval-augmented generation with OpenAI Agents SDK.

        Args:
            user_query: The user's question or query
            top_k: Number of content chunks to retrieve from Qdrant

        Returns:
            Generated response based on retrieved content
        """
        import time
        start_time = time.time()

        try:
            logger.info(f"Processing query: '{user_query}' with OpenAI Agents SDK")

            # Create a thread for the conversation
            thread = self.client.beta.threads.create()

            # Add the user's message to the thread
            self.client.beta.threads.messages.create(
                thread_id=thread.id,
                role="user",
                content=user_query
            )

            # Run the assistant
            run = self.client.beta.threads.runs.create(
                thread_id=thread.id,
                assistant_id=self.assistant.id,
                # Override the assistant instructions to use the specific query
                instructions=f"""You are an AI assistant that answers questions based only on the provided retrieved contexts.
                Do not use any prior knowledge or information outside of what is provided in the retrieved context.
                If the information is not present in the context, clearly state that you don't have that information.
                Always provide answers that are factual and based solely on the retrieved content.

                The user's question is: {user_query}

                Use the retrieve_book_content function to get relevant information from the book content, then answer the question based on that information."""
            )

            # Wait for the run to complete and handle tool calls
            final_response = self._wait_for_run_completion(thread.id, run.id, user_query, top_k)

            total_duration = time.time() - start_time
            logger.info(f"Successfully generated response in {total_duration:.2f}s")

            return final_response

        except Exception as e:
            total_duration = time.time() - start_time
            logger.error(f"Error processing query after {total_duration:.2f}s: {str(e)}")
            raise

    def _wait_for_run_completion(self, thread_id: str, run_id: str, original_query: str, top_k: int) -> str:
        """
        Wait for the assistant run to complete, handling any tool calls that occur.

        Args:
            thread_id: The ID of the thread
            run_id: The ID of the run
            original_query: The original user query
            top_k: Number of results to retrieve

        Returns:
            The final response from the assistant
        """
        import time
        from openai.types.beta.threads.run import RunStatus

        max_wait_time = 60  # Maximum wait time in seconds
        wait_interval = 1   # Interval between checks in seconds
        elapsed_time = 0

        while elapsed_time < max_wait_time:
            run = self.client.beta.threads.runs.retrieve(thread_id=thread_id, run_id=run_id)

            if run.status == "completed":
                # Get the messages from the thread to retrieve the assistant's response
                messages = self.client.beta.threads.messages.list(thread_id=thread_id, order="desc")
                for msg in messages.data:
                    if msg.role == "assistant":
                        # Extract the content from the message
                        content = ""
                        for item in msg.content:
                            if item.type == "text":
                                content += item.text.value
                        return content
                return "No response generated by the assistant."

            elif run.status == "requires_action":
                # Handle tool calls
                tool_calls = run.required_action.submit_tool_outputs.tool_calls
                tool_outputs = []

                for tool_call in tool_calls:
                    if tool_call.function.name == "retrieve_book_content":
                        # Parse the arguments
                        import json
                        args = json.loads(tool_call.function.arguments)
                        query = args.get("query", original_query)
                        k = args.get("top_k", top_k)

                        # Execute the retrieval
                        retrieved_content = self.retrieve_book_content(query, k)

                        # Add the output to the list
                        tool_outputs.append({
                            "tool_call_id": tool_call.id,
                            "output": retrieved_content
                        })

                # Submit the tool outputs
                self.client.beta.threads.runs.submit_tool_outputs(
                    thread_id=thread_id,
                    run_id=run_id,
                    tool_outputs=tool_outputs
                )

            elif run.status in ["failed", "cancelled", "expired"]:
                logger.error(f"Run failed with status: {run.status}")
                return f"Request failed with status: {run.status}"

            # Wait before checking again
            time.sleep(wait_interval)
            elapsed_time += wait_interval

        logger.error("Run timed out waiting for completion")
        return "Request timed out waiting for completion"

    def _format_retrieved_content(self, retrieved_chunks: List[Dict[str, Any]]) -> str:
        """
        Format retrieved content chunks into a context string for the LLM.

        Args:
            retrieved_chunks: List of retrieved content chunks

        Returns:
            Formatted context string
        """
        if not retrieved_chunks:
            return ""

        formatted_context = "## Retrieved Information from Book Content:\n\n"

        for i, chunk in enumerate(retrieved_chunks, 1):
            formatted_context += f"### Source {i}:\n"
            formatted_context += f"- **Title**: {chunk.get('document_title', 'Unknown')}\n"
            formatted_context += f"- **URL**: {chunk.get('source_url', 'N/A')}\n"
            formatted_context += f"- **Relevance Score**: {chunk.get('similarity_score', 0):.3f}\n"
            formatted_context += f"- **Content**: {chunk.get('content', '')}\n\n"

        return formatted_context


    def query_with_provenance(self, user_query: str, top_k: int = 5) -> Dict[str, Any]:
        """
        Process a user query and return both the answer and provenance information.

        Args:
            user_query: The user's question or query
            top_k: Number of content chunks to retrieve from Qdrant

        Returns:
            Dictionary containing the answer and provenance information
        """
        import time
        start_time = time.time()

        try:
            logger.info(f"Processing query with provenance: '{user_query}'")

            # For provenance, we'll first retrieve the content manually to track it
            retrieved_chunks = self.retrieve_content(user_query, top_k=top_k)

            if not retrieved_chunks:
                # Handle case where no relevant content is found
                total_duration = time.time() - start_time
                logger.info(f"Query processed in {total_duration:.2f}s (no content found)")

                return {
                    "answer": "I don't have information about this in the available book content.",
                    "sources": [],
                    "retrieved_chunks": [],
                    "performance": {
                        "total_duration": total_duration,
                        "retrieve_duration": total_duration,  # All time spent on retrieval
                        "generate_duration": 0
                    }
                }

            # Now use the regular query method to get the response
            answer = self.query(user_query, top_k)

            # Prepare provenance information
            sources = []
            for chunk in retrieved_chunks:
                source_info = {
                    "title": chunk.get('document_title', 'Unknown'),
                    "url": chunk.get('source_url', 'N/A'),
                    "similarity_score": chunk.get('similarity_score', 0),
                    "chunk_id": chunk.get('chunk_id', ''),
                    "document_id": chunk.get('document_id', '')
                }
                sources.append(source_info)

            total_duration = time.time() - start_time

            result = {
                "answer": answer,
                "sources": sources,
                "retrieved_chunks": retrieved_chunks,
                "performance": {
                    "total_duration": total_duration,
                    "retrieve_duration": 0,  # Would need to track this separately
                    "generate_duration": total_duration  # Approximation
                }
            }

            logger.info(f"Successfully generated response with provenance in {total_duration:.2f}s")
            return result

        except Exception as e:
            total_duration = time.time() - start_time
            logger.error(f"Error processing query with provenance after {total_duration:.2f}s: {str(e)}")
            raise


def main():
    """
    Main function to demonstrate the RAG Agent functionality.
    """
    print("=" * 70)
    print("AI AGENT WITH RETRIEVAL-AUGMENTED GENERATION")
    print("=" * 70)

    # Check environment configuration
    print("Environment configuration:")
    print(f"  OPENAI_API_KEY configured: {'Yes' if os.getenv('OPENAI_API_KEY') else 'No'}")
    print(f"  COHERE_API_KEY configured: {'Yes' if os.getenv('COHERE_API_KEY') else 'No'}")
    print(f"  QDRANT_URL configured: {'Yes' if os.getenv('QDRANT_URL') else 'No'}")
    print()

    try:
        # Initialize the RAG Agent
        print("Initializing RAG Agent...")
        agent = RAGAgent()
        print("[OK] RAG Agent initialized successfully!")
        print()

        # Example queries to demonstrate functionality
        example_queries = [
            "What is cognitive planning?",
            "Explain module 1 content",
            "How to create a document?",
            "Tutorial basics information",
            "Voice to action concepts"
        ]

        print(f"Demonstrating RAG Agent with {len(example_queries)} example queries:")
        print("-" * 70)

        for i, query in enumerate(example_queries, 1):
            print(f"\n{i}. Query: '{query}'")
            print("-" * 40)

            try:
                # Get response with provenance
                result = agent.query_with_provenance(query, top_k=3)

                print(f"Answer: {result['answer'][:500]}{'...' if len(result['answer']) > 500 else ''}")

                print(f"\nSources ({len(result['sources'])} retrieved):")
                for j, source in enumerate(result['sources'][:2], 1):  # Show first 2 sources
                    print(f"  {j}. Title: {source['title'][:50]}{'...' if len(source['title']) > 50 else ''}")
                    print(f"     URL: {source['url']}")
                    print(f"     Similarity: {source['similarity_score']:.3f}")

                if len(result['sources']) > 2:
                    print(f"  ... and {len(result['sources']) - 2} more sources")

                print()

            except Exception as e:
                print(f"  Error processing query: {str(e)}")
                print()

        print("=" * 70)
        print("RAG AGENT DEMONSTRATION COMPLETE")
        print("=" * 70)

    except Exception as e:
        logger.error(f"Error in main function: {str(e)}")
        print(f"[ERROR] Failed to initialize RAG Agent: {str(e)}")


def run_interactive_mode():
    """
    Run the RAG Agent in interactive mode for real-time queries.
    """
    print("\nINTERACTIVE MODE")
    print("Enter queries to ask the RAG Agent (type 'quit' to exit):")

    try:
        # Initialize the RAG Agent
        agent = RAGAgent()
        print("RAG Agent ready! Enter your queries below:")

        while True:
            user_input = input("\nEnter your question (or 'quit' to exit): ").strip()

            if user_input.lower() in ['quit', 'exit', 'q', '']:
                print("Exiting interactive mode...")
                break

            try:
                print("\nRetrieving relevant information...")
                result = agent.query_with_provenance(user_input, top_k=3)

                print(f"\nAnswer: {result['answer']}")

                if result['sources']:
                    print(f"\nSources used ({len(result['sources'])} documents):")
                    for i, source in enumerate(result['sources'], 1):
                        print(f"  {i}. {source['title']}")
                        print(f"     URL: {source['url']}")
                        print(f"     Relevance: {source['similarity_score']:.3f}")

            except Exception as e:
                print(f"Error processing query: {str(e)}")

    except Exception as e:
        print(f"Error initializing RAG Agent: {str(e)}")


if __name__ == "__main__":
    import sys

    # Check if running in interactive mode
    if len(sys.argv) > 1 and sys.argv[1] == "--interactive":
        run_interactive_mode()
    else:
        main()