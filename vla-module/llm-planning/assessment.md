---
sidebar_position: 7
---

# LLM Planning Assessment

This assessment evaluates understanding of LLM-based cognitive planning concepts, including natural language to action sequences, task decomposition strategies, autonomous behavior preparation, and best practices for implementation.

## Knowledge Check Questions

1. What does chain-of-thought prompting encourage LLMs to do when translating natural language to action sequences?
   - A) Generate shorter responses
   - B) Explicitly show their reasoning process, breaking complex tasks into sequential steps
   - C) Only use simple actions
   - D) Eliminate the need for planning
   **Correct Answer: B**
   **Explanation: Chain-of-thought prompting encourages LLMs to explicitly show their reasoning process, breaking complex tasks into sequential steps and reasoning about relationships between them.**

2. What is the ReAct framework in LLM planning?
   - A) A framework for reading and acting
   - B) A framework combining reasoning and acting in a unified approach
   - C) A framework for remembering and acting
   - D) A framework for repeating and acting
   **Correct Answer: B**
   **Explanation: The ReAct framework combines reasoning and acting in a unified approach, alternating between reasoning about the task and taking actions.**

3. What are Hierarchical Task Networks (HTNs) used for in task decomposition?
   - A) To eliminate the need for subtasks
   - B) To organize complex goals into structured hierarchies of subtasks
   - C) To create random task sequences
   - D) To only handle simple tasks
   **Correct Answer: B**
   **Explanation: HTNs organize complex goals into structured hierarchies of subtasks, with each subtask potentially decomposable until reaching primitive actions.**

4. What is an advantage of behavior trees in task decomposition?
   - A) They only handle simple tasks
   - B) They provide clear visualization of task structure and built-in support for complex conditional logic
   - C) They eliminate the need for planning
   - D) They only work for navigation tasks
   **Correct Answer: B**
   **Explanation: Behavior trees provide clear visualization of task structure and have built-in support for complex conditional logic, making them well-suited for handling failures and recovery.**

5. What is important for safety-first design in LLM-based planning?
   - A) Only focusing on performance
   - B) Embedding safety constraints into planning, implementing validation, and maintaining human oversight
   - C) Eliminating safety checks to improve speed
   - D) Only using hardware safety mechanisms
   **Correct Answer: B**
   **Explanation: Safety-first design requires embedding constraints into planning, validation layers, and maintaining human oversight mechanisms.**

## Application Questions

6. In the domestic task planning example with "Please clean the living room, put the books back on the shelf, and take the trash to the bin outside," what does the LLM identify?
   - A) Only cleaning tasks
   - B) Multiple subgoals: cleaning, organizing books, and disposing of trash
   - C) Only trash disposal
   - D) Only book organizing
   **Correct Answer: B**
   **Explanation: The LLM identifies multiple subgoals: cleaning the living room, organizing books, and disposing of trash, then decomposes these into actionable steps.**

7. What does program-of-thought generation produce in LLM planning?
   - A) Only simple actions
   - B) Structured action descriptions or executable code from natural language
   - C) Only natural language explanations
   - D) Only visual outputs
   **Correct Answer: B**
   **Explanation: Program-of-thought generation creates structured action descriptions or executable code from natural language, providing more precise action specifications.**

8. What do neural task planning approaches offer?
   - A) They require no training data
   - B) They can handle novel tasks not explicitly programmed and learn from data
   - C) They are always predictable
   - D) They eliminate the need for safety mechanisms
   **Correct Answer: B**
   **Explanation: Neural task planning can handle novel tasks not explicitly programmed and can learn complex decomposition patterns from data, with potential for continuous improvement.**

## Scenario-Based Questions

9. What is a key benefit of simulation-based training for autonomous humanoid behavior?
   - A) It eliminates the need for safety
   - B) It provides a risk-free environment for testing planning strategies without risk to physical systems or humans
   - C) It makes robots faster
   - D) It reduces computational requirements
   **Correct Answer: B**
   **Explanation: Simulation provides a safe and cost-effective environment for testing planning strategies without risk to physical systems or humans.**

10. What does the "reality gap" refer to in simulation-based training?
    - A) The difference between different simulations
    - B) The differences between simulated and real environments that need to be addressed
    - C) The time difference between simulation and reality
    - D) The cost difference between simulation and reality
    **Correct Answer: B**
    **Explanation: The reality gap refers to the differences between simulated and real environments that need to be addressed for effective transfer of learned behaviors.**

11. What educational approach should be prioritized when developing LLM-based planning for educational purposes?
    - A) Maximum technical complexity
    - B) Explaining concepts before implementation details and ensuring accessibility
    - C) Advanced algorithms only
    - D) Hardware specifications
    **Correct Answer: B**
    **Explanation: Educational applications should prioritize explaining concepts before implementation details and ensure content is accessible to students with appropriate background knowledge.**

12. What do all the LLM planning examples demonstrate?
    - A) Only simple tasks
    - B) Hierarchical decomposition, context integration, multi-modal planning, and safety considerations
    - C) Only navigation tasks
    - D) Only manipulation tasks
    **Correct Answer: B**
    **Explanation: All examples demonstrate hierarchical decomposition of complex goals, context integration, multi-modal planning, and appropriate safety considerations for their domain.**

[Next: Module Summary](../index.md)