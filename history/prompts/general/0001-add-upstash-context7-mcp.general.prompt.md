---
id: 0001
title: Add Upstash Context7 MCP
stage: general
date: 2025-12-06
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: master
user: unknown
command: mcp add --transport stdio context7 npx @upstash/context7-mcp
labels: ["mcp", "setup", "error"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - none
tests:
  - none
---

## Prompt

mcp add --transport stdio context7 npx @upstash/context7-mcp

## Response snapshot

{"Bash_response": {"result": "Exit code 2\nUsage: mcp [OPTIONS] COMMAND [ARGS]...\r\nTry 'mcp --help' for help.\r\n+- Error ---------------------------------------------------------------------+\r\n| No such command 'add'.                                                      |\r\n+-----------------------------------------------------------------------------+"}}

## Outcome

- ✅ Impact: Attempted to add an MCP server, but the `mcp` command did not have an 'add' subcommand.
- 🧪 Tests: none
- 📁 Files: none
- 🔁 Next prompts: Proceed with user's new request to create a Docusaurus project.
- 🧠 Reflection: The `mcp` command usage was incorrect. Need to verify `mcp` subcommands before use.

## Evaluation notes (flywheel)

- Failure modes observed: `mcp` command failed with "No such command 'add'". This indicates an incorrect usage of the `mcp` CLI.
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Investigate correct `mcp` command for adding servers or available `mcp` commands.
