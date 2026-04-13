# EvoEngine Expert Agent Usage Guide

## Available Expert Commands

I've created two specialized expert agents for working on the EvoEngine sorghum PAR calculation project:

### 1. `/evoengine-expert` - C++ & Python Bindings Expert

**Use this when:**
- Adding new C++ APIs to EvoEngine
- Creating or debugging Python bindings with pybind11
- Working with the EvoEngine Entity-Component-System (ECS)
- Debugging build issues with CMake
- Implementing illumination calculation features
- Integrating ray tracing functionality

**Expertise includes:**
- Modern C++ (C++17/20)
- pybind11 binding patterns
- EvoEngine architecture (Scene, Entity, Component, Layer)
- CMake build configuration
- GPU ray tracing integration
- Python-C++ debugging

**Example usage:**
```
/evoengine-expert

I'm getting a compilation error when trying to add the IlluminationEstimationOnSorghum function.
The error says 'SorghumLayer' was not declared. How do I fix this?
```

### 2. `/c4-expert` - C4 Photosynthesis & Sorghum Specialist

**Use this when:**
- Validating PAR calculation results
- Interpreting photosynthesis data
- Checking if values are biologically reasonable
- Understanding sorghum physiology and light response
- Linking ray tracing outputs to biological outcomes
- Estimating photosynthesis from PAR data

**Expertise includes:**
- C4 photosynthesis biochemistry (NADP-ME pathway)
- Sorghum physiological parameters
- Light interception and PAR validation
- Photo3 model integration
- Field measurement interpretation
- Growth stage characteristics

**Example usage:**
```
/c4-expert

I calculated a daily PAR integral of 45 mol m⁻² day⁻¹ for a sorghum leaf in Maricopa, AZ
on June 13. The peak PAR was 1950 μmol m⁻² s⁻¹. Does this seem reasonable?
```

## Quick Start Examples

### Scenario 1: Adding Missing APIs

```
/evoengine-expert

I need to add the GetAllIlluminationEstimationResultsOnSorghum() function to the Python bindings.
The test script says it's missing. Can you walk me through the exact steps?
```

The agent will guide you through:
1. Adding the C++ implementation
2. Adding the header declaration
3. Adding the pybind11 binding
4. Rebuilding the project
5. Verifying the API is available

### Scenario 2: Validating Results

```
/c4-expert

Here are my PAR results:
- Peak PAR: 2100 μmol m⁻² s⁻¹
- Daily integral: 52 mol m⁻² day⁻¹
- Location: 33°N latitude
- Date: June 13 (summer solstice period)
- Weather: Clear day

Is this within expected ranges for sorghum?
```

The agent will validate against:
- Physical constraints
- Biological expectations
- Literature values
- Seasonal/location norms

### Scenario 3: Debugging Build Issues

```
/evoengine-expert

CMake build is failing with linker errors after I added the new illumination functions.
Error message: "undefined reference to `SorghumLayer::CalculateIllumination()'"

What could be wrong?
```

The agent will help troubleshoot:
- Missing library dependencies
- CMakeLists.txt configuration
- Include paths
- Linking order

### Scenario 4: Understanding Results

```
/c4-expert

My PAR calculation shows high spatial variation (std dev = 300 μmol m⁻² s⁻¹) on a single leaf.
The mean PAR is 1500 μmol m⁻² s⁻¹. Is this amount of variation expected?
```

The agent will explain:
- Self-shading effects
- Leaf curvature impact
- Expected variation ranges
- Biological significance

## Tips for Using the Agents

1. **Be specific about your context**: Include error messages, values, file paths
2. **Combine agents**: Use `/evoengine-expert` for implementation, then `/c4-expert` for validation
3. **Ask follow-up questions**: The agents maintain context and can dive deeper
4. **Share actual data**: Provide real PAR values, file contents, or error logs
5. **Request examples**: Ask for code examples following EvoEngine patterns

## Reference Materials

Both agents have access to information from:
- `TRANSFER_TO_EVOENGINE_MACHINE.md` - Complete transfer guide
- `README_EVOENGINE_PAR_ADAPTATION.md` - Adaptation instructions
- EvoEngine codebase patterns
- Scientific literature on C4 photosynthesis and sorghum

## Workflow Integration

**Typical workflow:**

1. **Initial setup**: Use `/evoengine-expert` to verify build and dependencies
2. **API implementation**: Use `/evoengine-expert` to add missing functions
3. **Testing**: Run compatibility test and PAR calculation scripts
4. **Result validation**: Use `/c4-expert` to check if values are reasonable
5. **Debugging**: Use `/evoengine-expert` for technical issues, `/c4-expert` for biological interpretation
6. **Iteration**: Refine based on validation feedback

## How It Works

These are **slash commands** stored in `.claude/commands/` directory:
- `evoengine-expert.md` - Technical expert prompt
- `c4-expert.md` - Scientific expert prompt

When you type `/evoengine-expert` or `/c4-expert`, Claude activates that specialized expertise mode with deep knowledge of:
- The project structure and goals
- C++ and Python binding patterns specific to EvoEngine
- Scientific validation criteria for sorghum PAR calculations
- Common issues and solutions for this specific workflow

## Need Help?

If you're unsure which agent to use:
- **Technical/coding questions**: `/evoengine-expert`
- **Scientific/validation questions**: `/c4-expert`
- **Both**: Ask one agent, and it will suggest using the other if needed

Happy coding! 🌱☀️
