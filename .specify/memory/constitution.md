<!--
Sync Impact Report
Version change: 0.0.0 -> 1.0.0 (MAJOR: Initial creation/significant update)
List of modified principles: All principles added/defined.
Added sections: Key Standards, Constraints, Operational Rules & Guardrails, Success Criteria, Deliverables
Removed sections: none
Templates requiring updates:
- .specify/templates/plan-template.md: ⚠ pending
- .specify/templates/spec-template.md: ⚠ pending
- .specify/templates/tasks-template.md: ⚠ pending
- .specify/templates/commands/sp.constitution.md: N/A (not found)
- .specify/templates/commands/sp.phr.md: N/A (not found)
Follow-up TODOs: none
-->
# Physical AI & Humanoid Robotics — AI-Native Textbook Constitution

## Core Principles

### Accuracy & Traceability
All factual claims MUST be verifiable and traceable to primary sources or high-quality secondary sources. Every non-trivial technical claim MUST include metadata pointing to the source (doc id, section, URL, citation).

### Pedagogical Clarity
Content MUST be clear for learners with a CS/Engineering background; concepts explained progressively from fundamentals to advanced topics with worked examples and exercises.

### Reproducibility
All code, simulations, and experiments MUST be runnable: provide code snippets, Docker or environment files, and test scripts that reproduce results.

### Spec-Driven Authoring
Use Spec-Kit Plus conventions for defining agent contracts, content specs, test specs, and CI workflows. Chapters, agents, and ingestion pipelines MUST be described as explicit specs.

### Ethics & Safety
Include a dedicated chapter on safety, ethics, standards, and human-robot interaction; require the chatbot to refuse unsafe/illegal instructions.

### Openness & Licensing
Default content license: CC-BY 4.0 for book content; MIT for source code examples (modifiable by maintainers).

### Accessibility & Localization
Content MUST be accessible (semantic HTML, alt text, readable fonts) and support English + Urdu translation toggles (option for Roman Urdu).

## Key Standards (must follow)

### Authoring & Tools
Primary authoring: Docusaurus (MDX) + Spec-Kit Plus templates.
Authoring automation: Claude Code for draft generation, subagents, and transformation tasks.

### Citation & Sourcing
Citation format: APA for chapter references and bibliography.
Minimum sources: 20 distinct high-quality sources across the book; at least 40% should be peer-reviewed or primary technical docs (papers, standards).
Every chapter MUST include a “Sources & Further Reading” section with full citations and links.

### Code & Repro
Include runnable examples: package.json / requirements.txt, Dockerfile, or GitHub Codespaces/devcontainer config.
Provide unit/integration test scripts for ingestion and chat endpoints.

### Vectorization & Retrieval
Embeddings provider: configurable (default OpenAI embeddings). Store metadata: doc_id, chapter_id, chunk_id, start_offset, end_offset, language.
Vector DB: Qdrant collection(s) with schema documented in repo.

### Auth & Personalization
Integrate Better-Auth for Signup/Signin.
Signup collects: role (student/professional), programming level, hardware experience, preferred language (English/Urdu/Roman-Urdu).
Personalization rules defined in spec: mapping from user profile to content variant (simplified, standard, advanced).

### Chatbot Behavior
System instruction MUST enforce: “Answer using ONLY the provided retrieved contexts. If the information is not present, say ‘I don’t know’ and suggest where to look.”
Support two retrieval modes: (a) global (book-wide), (b) restricted-to-selection (only selected text).
Chat outputs MUST include citations (chapter and chunk id) and confidence score metadata.

### Quality
Plagiarism tolerance: 0% before final submission.
Readability guideline: aim for Flesch-Kincaid grade level 10–14 for main text; exercise language can be more technical.

## Constraints

### Platform & Deployment
Site built with Docusaurus and deployable to GitHub Pages (gh-pages branch or GH Action).
Backend (chat/api) MUST be deployable to a cloud container or serverless platform (Render, Fly, Vercel Serverless functions, etc.).

### Size & Structure
Minimum 8 chapters covering: Intro, Kinematics, Dynamics, Actuation & Control, Perception & Sensing, Human-Robot Interaction & Safety, Ethics & Regulation, Projects & Labs.


### RAG Requirements
Ingestion pipeline MUST chunk docs, store embeddings in Qdrant, and expose indices for selection-limited retrieval.
Selected-text answering MUST be provably restricted: either by passing only selected text vectors to the agent or by using Qdrant filters that strictly limit retrieval to selection metadata.

### Time / Resource
Use the free tiers where possible (Qdrant Cloud free tier, Neon Serverless for Postgres).
Ensure API keys and secrets are kept out of repo; use GitHub Secrets and a documented .env.example.

### Localization
Provide machine-generated Urdu translation for each chapter and option for Roman Urdu rendering.
Cache translations to avoid repeated calls.

## Governance

### Operational Rules & Guardrails
Privacy: Do not store plain text of user secrets or PII in vector DB. Store only metadata pointers and hashed identifiers where necessary.
Safety: Agents MUST include refusal behavior for unsafe requests (weapons, illegal manipulation, unethical actions). Document safety policy and show test cases.
Traceability: Every generated agent answer MUST return its retrieval provenance (doc_id, chapter, chunk_id) and the snippet used.
Plagiarism & IP: Verify generated content via plagiarism checks before final submission; attribute third-party diagrams/code per license.


### Success Criteria (Acceptance Tests)
Base (required for 100 points)
Book repo follows Spec-Kit Plus layout and includes /spec definitions for agents and ingestion pipelines.
Docusaurus book builds and is deployed to GitHub Pages with working navigation and at least 8 chapters.
Ingestion pipeline successfully indexes the book into Qdrant; an ingestion test script is present and passes.
Embedded RAG chatbot responds to queries using book content and returns citations.
Selected-text answering works: selecting text in the UI and asking a question returns answers based only on that selected content (test script demonstrates this).

Bonus (each up to +50 points)
Claude Code Subagents / Agent Skills: At least two documented subagents (e.g., summary_agent, exercise_generator) implemented and callable via agent spec.
Better-Auth Signup/Signin: Integrated; collects and stores profile fields in Neon Postgres; signup flow documented and testable.
Per-Chapter Personalization: Logged-in user can press a button to apply personalized content variant (e.g., simplified examples) for that chapter.
Per-Chapter Urdu Translation: Button toggles chapter to Urdu; Roman Urdu option available.
Translations cached and reversible.

Non-functional
Chat latency under reasonable bounds (configurable; document target e.g., < 4s average retrieval+response for small queries).
All tests in EVAL.md pass (ingest/test/chat/test-selection/auth).

README contains a 30–60s demo GIF or video and a live deployment link.

### Deliverables (required in repo)
README.md — project overview, live link, demo GIF/video.
docs/ — Docusaurus MDX chapters (minimum 8).
spec/ — Spec-Kit Plus specs for agents, ingestion, and tests (/sp.* files).
backend/ — FastAPI (or equivalent) backend with /ingest, /chat, /user endpoints and tests.
infra/ — GH Actions workflow for build & deploy; Dockerfile or deployment scripts.
DEPLOY.md — step-by-step deployment guide (how to set secrets, deploy backend, create Qdrant & Neon instances).
EVAL.md — acceptance test instructions & scripts.
LICENSE — CC-BY 4.0 for content, MIT for code (default).
TRANSLATIONS/ — cached Urdu translations (optional at submission).
demo/ — short demo video/GIF and test output logs.


**Version**: 1.0.0 | **Ratified**: 2025-12-06 | **Last Amended**: 2025-12-06
