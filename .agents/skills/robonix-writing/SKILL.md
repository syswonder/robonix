---
name: robonix-writing
description: Write or edit commit messages, PR and issue descriptions, READMEs and code comments for Robonix so they read like a person wrote them. Use whenever producing prose that goes into the repository or onto GitHub.
---

# robonix-writing

Follow the writing page of `.agents/guidelines/`. In short:

1. **Say the fact first.** One claim per sentence. No introduction, no summary line at the end.
2. **Cut the generated-text habits:** "not X but Y" contrasts used for emphasis, one-line closers, staged openers, lists of exactly three, a dash in every sentence, bold on every item, headings that restate the text below them, and inflated words (*crucial, robust, seamless, leverage, delve, pivotal, comprehensive*).
3. **Keep only what the reader lacks.** A commit says why and what; a PR says problem, change, verification; a comment says why the code cannot speak for itself. Drop anything the diff already shows.
4. **Don't invent.** No numbers, names or results you did not measure or read.
5. **Format for the destination.** Commit bodies wrapped at 72 columns; PR and issue bodies one paragraph per line; English everywhere in the repository; no agent named as author or co-author (`human-authorship`).

Read the draft once more and delete every sentence that would not be missed.
