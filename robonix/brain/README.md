# Brain

Central runtime of the embodied system. Responsible for:

- Interpreting commands
- Task decomposition into Skills
- Planning under Cost constraints
- Reading/writing from Memory
- Calling LLM/VLM for reasoning

## 🧠 Includes:

- World model abstraction
- Model interfaces (e.g., via Ollama or DeepSeek)
- Prompt construction (JIT / dynamic)
- Skill orchestration logic

This is the decision-making and planning center of the system.

requirements.txt contains dependencies for the brain module.(as a example)

add .env file in /brain directory with the following content:

```plaintext
API_KEY=sk-xxx
```

Test CMD as follow :

```SHELL
python  path/to/brain-deepseek.py
```
