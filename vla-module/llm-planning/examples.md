---
sidebar_position: 6
---

# LLM Planning Examples

This section presents practical examples that demonstrate LLM-based cognitive planning in robotic applications. These examples illustrate how LLMs interpret natural language goals and decompose them into executable action sequences.

## Example 1: Domestic Task Planning

### Scenario Description
A domestic robot receives a complex task: "Please clean the living room, put the books back on the shelf, and take the trash to the bin outside."

### LLM Planning Process
- **Goal Analysis**: The LLM identifies multiple subgoals: cleaning, organizing books, and disposing of trash
- **Task Decomposition**:
  1. Survey the living room to identify cleaning needs
  2. Locate scattered books and plan their return to the shelf
  3. Identify trash items and plan their disposal
  4. Navigate outside to the external bin
- **Action Sequencing**: The LLM generates a sequence of robotic actions for each subtask
- **Context Integration**: The LLM considers the current state of the room and available resources

### Key Planning Aspects
- Multi-goal handling with prioritization
- Spatial reasoning for navigation and object placement
- Object recognition and manipulation planning
- Integration of cleaning, organizing, and transportation tasks

## Example 2: Warehouse Logistics Planning

### Scenario Description
A warehouse robot receives: "Move all boxes of product X-450 from shelf A3 to the packing station and update inventory."

### LLM Planning Process
- **Goal Analysis**: Identify the specific product, source location, destination, and inventory update requirement
- **Task Decomposition**:
  1. Navigate to shelf A3
  2. Identify all boxes of product X-450
  3. Transport boxes to packing station in optimal sequence
  4. Update inventory system with location changes
- **Resource Optimization**: Plan efficient paths and handle multiple boxes
- **System Integration**: Coordinate with warehouse management systems

### Key Planning Aspects
- Inventory management integration
- Path optimization for multiple destinations
- Quality control during item handling
- System-level coordination with warehouse operations

## Example 3: Educational Demonstration Planning

### Scenario Description
An educational robot receives: "Demonstrate how to build a pyramid with these blocks and explain the geometric principles."

### LLM Planning Process
- **Goal Analysis**: Identify construction task and educational explanation requirement
- **Task Decomposition**:
  1. Select appropriate blocks for pyramid construction
  2. Plan the construction sequence layer by layer
  3. Position self appropriately for student viewing
  4. Generate educational explanations for each step
- **Pedagogical Integration**: Incorporate educational objectives into the action sequence
- **Safety Considerations**: Plan safe manipulation of blocks around students

### Key Planning Aspects
- Educational objective integration
- Multi-modal output (action and explanation)
- Safety in educational environments
- Adaptive complexity based on student level

## Example 4: Healthcare Assistance Planning

### Scenario Description
A healthcare robot receives: "Assist patient in room 205 with medication and report their status to the nurse station."

### LLM Planning Process
- **Goal Analysis**: Identify patient location, medication task, and reporting requirement
- **Task Decomposition**:
  1. Navigate to room 205
  2. Verify patient identity and medication requirements
  3. Safely assist with medication intake
  4. Assess patient condition and generate status report
  5. Communicate with nurse station
- **Safety Integration**: Multiple safety checks throughout the process
- **Privacy Considerations**: Handle sensitive health information appropriately

### Key Planning Aspects
- Healthcare safety protocols
- Privacy and confidentiality
- Multi-step task coordination
- Communication with healthcare systems

## Example 5: Manufacturing Quality Planning

### Scenario Description
A manufacturing robot receives: "Inspect the assembly on station 7, identify any defects, and if found, disassemble and restart the process."

### LLM Planning Process
- **Goal Analysis**: Inspection, defect identification, and potential rework
- **Task Decomposition**:
  1. Navigate to station 7
  2. Perform detailed visual inspection
  3. Identify and classify any defects
  4. If defects exist, plan disassembly sequence
  5. Restart the assembly process
  6. Verify successful completion
- **Quality Control**: Integrate quality standards into the planning process
- **Adaptive Response**: Different actions based on inspection results

### Key Planning Aspects
- Quality control integration
- Conditional planning based on inspection results
- Manufacturing safety protocols
- Process standardization and repeatability

## Common Planning Patterns

### Hierarchical Decomposition
All examples demonstrate breaking complex goals into hierarchical subtasks that can be managed and executed systematically.

### Context Integration
Each example shows how the LLM incorporates environmental context (location, available objects, system states) into the planning process.

### Multi-Modal Planning
The examples illustrate planning that integrates different types of actions (navigation, manipulation, communication, etc.) into coherent sequences.

### Safety and Validation
All examples include safety considerations and validation steps appropriate to their domain.

## Assessment Questions

1. In the domestic task planning example, what does the LLM identify when given "Please clean the living room, put the books back on the shelf, and take the trash to the bin outside"?
   - A) Only cleaning tasks
   - B) Multiple subgoals: cleaning, organizing books, and disposing of trash
   - C) Only trash disposal
   - D) Only book organizing
   **Correct Answer: B**
   **Explanation: The LLM identifies multiple subgoals: cleaning the living room, organizing books, and disposing of trash, then decomposes these into actionable steps.**

2. What is a key aspect of the educational demonstration planning example?
   - A) Only construction tasks
   - B) Educational objective integration, multi-modal output, and safety in educational environments
   - C) Only safety considerations
   - D) Only explanation generation
   **Correct Answer: B**
   **Explanation: The educational example integrates educational objectives, provides multi-modal output (action and explanation), and considers safety in educational environments.**

3. What do all the LLM planning examples demonstrate?
   - A) Only simple tasks
   - B) Hierarchical decomposition, context integration, multi-modal planning, and safety considerations
   - C) Only navigation tasks
   - D) Only manipulation tasks
   **Correct Answer: B**
   **Explanation: All examples demonstrate hierarchical decomposition of complex goals, context integration, multi-modal planning, and appropriate safety considerations for their domain.**

[Next: LLM Planning Assessment](./assessment.md)