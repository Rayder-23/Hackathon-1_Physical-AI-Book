# Research: Integrated RAG Chatbot SDK Alignment

## OpenAI Agents SDK Integration

**Decision**: OpenAI Agents SDK is compatible with OpenRouter API through OpenAI-compatible interfaces
**Rationale**: OpenRouter supports the OpenAI API specification, allowing Agents SDK to work with minimal modifications
**Alternatives considered**:
- Custom orchestration framework
- LangChain agents
- Anthropic Claude with tool use

**Findings**:
- OpenRouter supports agent functions and tool calling
- Some advanced features may have limitations
- Rate limits need to be considered for agent loops

## ChatKit SDK Integration

**Decision**: ChatKit SDK can be embedded in static sites using standard JavaScript inclusion
**Rationale**: ChatKit is designed for client-side integration with minimal backend dependencies
**Alternatives considered**:
- Custom React chat components
- Third-party chat libraries (like Stream Chat)
- Simple textarea-based interface

**Findings**:
- Requires authentication token management
- Custom styling possible through CSS variables
- Message streaming works well with static sites

## Qwen Embeddings Performance

**Decision**: Qwen embeddings provide good cost-performance balance for book content
**Rationale**: Lower cost than OpenAI embeddings with acceptable quality for book content
**Alternatives considered**:
- OpenAI text-embedding-3-small
- Sentence Transformers (local)
- Cohere embeddings

**Findings**:
- Quality is acceptable for RAG on technical content
- May require more aggressive chunking for optimal results
- Cost savings significant for large book content

## Qdrant Cloud Free Tier Limitations

**Decision**: Free tier provides sufficient capacity for book content indexing
**Rationale**: Most technical books fit within free tier limits with proper chunking
**Alternatives considered**:
- Pinecone free tier
- Weaviate Cloud
- Local vector storage

**Findings**:
- 1 free collection with 5GB storage
- 1M vectors limit is sufficient for most books
- Query per second limits adequate for non-enterprise usage

## OpenRouter API Compatibility

**Decision**: OpenRouter provides good OpenAI API compatibility for agent workflows
**Rationale**: Supports the required endpoints for agent orchestration
**Alternatives considered**:
- Direct OpenAI API
- Anthropic API
- Perplexity API

**Findings**:
- Most OpenAI endpoints supported
- Some models may have different capabilities
- Rate limits and pricing vary by model