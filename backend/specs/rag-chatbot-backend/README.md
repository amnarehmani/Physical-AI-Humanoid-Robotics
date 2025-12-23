# RAG Chatbot Backend Feature

This directory contains the specifications and artifacts for implementing the RAG (Retrieval-Augmented Generation) Chatbot Backend for the AI-Spec-Driven Book Workspace.

## Purpose

The RAG Chatbot Backend enables users to ask questions about Docusaurus-published book content and receive accurate, grounded answers based exclusively on the book's information, with no hallucinations or use of external knowledge.

## Key Features

- Content ingestion from Markdown/MDX book sources
- Vector-based retrieval using Qdrant Cloud
- Grounded question answering with minimal hallucination
- Support for both global book queries and selected-text-only queries
- Integration with Cohere for LLM inference

## Directories

- `specs/`: Feature specifications
- `checklists/`: Validation checklists
- `plans/`: Architectural plans (to be created)
- `tasks/`: Implementation tasks (to be created)

## API Endpoints

- `POST /ingest`: Ingest book content
- `POST /query`: Query book content globally
- `POST /query/selection`: Query using selected text only